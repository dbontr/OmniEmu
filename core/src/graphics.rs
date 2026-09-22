use std::collections::{BTreeMap, VecDeque};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ShaderStage {
    Vertex,
    Fragment,
    Compute,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrimitiveTopology {
    Points,
    Lines,
    Triangles,
    TriangleStrip,
    Quads,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PixelFormat {
    Rgba8,
    Bgra8,
    Rgb565,
    Rgba5551,
    Depth16,
    Depth24Stencil8,
    Rgba16Float,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GpuCommand {
    BeginFrame,
    SetRenderTarget {
        address: u64,
        width: u32,
        height: u32,
        format: PixelFormat,
    },
    Upload {
        address: u64,
        bytes: Vec<u8>,
    },
    Draw {
        topology: PrimitiveTopology,
        first: u32,
        count: u32,
    },
    Dispatch {
        x: u32,
        y: u32,
        z: u32,
    },
    Barrier,
    EndFrame,
}
#[derive(Debug, Clone)]
pub struct GpuCommandQueue {
    commands: VecDeque<GpuCommand>,
    upload_bytes: usize,
    upload_budget: usize,
}

impl GpuCommandQueue {
    pub fn new(upload_budget: usize) -> Self {
        Self {
            commands: VecDeque::new(),
            upload_bytes: 0,
            upload_budget,
        }
    }

    pub fn len(&self) -> usize {
        self.commands.len()
    }
    pub fn is_empty(&self) -> bool {
        self.commands.is_empty()
    }
    pub fn queued_upload_bytes(&self) -> usize {
        self.upload_bytes
    }

    pub fn push(&mut self, command: GpuCommand) -> Result<(), String> {
        let bytes = match &command {
            GpuCommand::Upload { bytes, .. } => bytes.len(),
            _ => 0,
        };
        if self.upload_bytes.saturating_add(bytes) > self.upload_budget {
            return Err("GPU upload queue exceeds its configured memory budget".into());
        }
        self.upload_bytes += bytes;
        self.commands.push_back(command);
        Ok(())
    }

    pub fn pop(&mut self) -> Option<GpuCommand> {
        let command = self.commands.pop_front()?;
        if let GpuCommand::Upload { ref bytes, .. } = command {
            self.upload_bytes -= bytes.len();
        }
        Some(command)
    }

    pub fn clear(&mut self) {
        self.commands.clear();
        self.upload_bytes = 0;
    }
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TranslatedShader {
    pub guest_hash: u64,
    pub stage: ShaderStage,
    pub source_generation: u64,
    pub webgpu_source: String,
}

#[derive(Debug, Default)]
pub struct ShaderCache {
    shaders: BTreeMap<(ShaderStage, u64), TranslatedShader>,
}

impl ShaderCache {
    pub fn insert(&mut self, shader: TranslatedShader) {
        self.shaders
            .insert((shader.stage, shader.guest_hash), shader);
    }

    pub fn get(
        &self,
        stage: ShaderStage,
        guest_hash: u64,
        source_generation: u64,
    ) -> Option<&TranslatedShader> {
        self.shaders
            .get(&(stage, guest_hash))
            .filter(|shader| shader.source_generation == source_generation)
    }

    pub fn invalidate_generation(&mut self, source_generation: u64) {
        self.shaders
            .retain(|_, shader| shader.source_generation == source_generation);
    }

    pub fn clear(&mut self) {
        self.shaders.clear();
    }
    pub fn len(&self) -> usize {
        self.shaders.len()
    }
    pub fn is_empty(&self) -> bool {
        self.shaders.is_empty()
    }
}

pub trait ShaderTranslator {
    fn translate(&mut self, stage: ShaderStage, guest_code: &[u8]) -> Result<String, String>;
}

#[derive(Debug, Default)]
pub struct ShaderPipeline {
    cache: ShaderCache,
}

impl ShaderPipeline {
    pub fn cache(&self) -> &ShaderCache {
        &self.cache
    }

    pub fn clear(&mut self) {
        self.cache.clear();
    }

    pub fn source_changed(&mut self, source_generation: u64) {
        self.cache.invalidate_generation(source_generation);
    }

    pub fn resolve<'a, T: ShaderTranslator>(
        &'a mut self,
        stage: ShaderStage,
        guest_hash: u64,
        source_generation: u64,
        guest_code: &[u8],
        translator: &mut T,
    ) -> Result<&'a TranslatedShader, String> {
        if self
            .cache
            .get(stage, guest_hash, source_generation)
            .is_none()
        {
            let webgpu_source = translator.translate(stage, guest_code)?;
            if webgpu_source.trim().is_empty() {
                return Err("shader translator produced empty WebGPU source".into());
            }
            self.cache.insert(TranslatedShader {
                guest_hash,
                stage,
                source_generation,
                webgpu_source,
            });
        }
        self.cache
            .get(stage, guest_hash, source_generation)
            .ok_or_else(|| "translated shader disappeared from the shader cache".to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn upload_budget_bounds_command_memory() {
        let mut queue = GpuCommandQueue::new(1024);
        queue
            .push(GpuCommand::Upload {
                address: 0x1000,
                bytes: vec![1; 768],
            })
            .unwrap();
        assert!(queue
            .push(GpuCommand::Upload {
                address: 0x2000,
                bytes: vec![2; 512]
            })
            .is_err());
        assert_eq!(queue.queued_upload_bytes(), 768);
        queue.pop();
        assert_eq!(queue.queued_upload_bytes(), 0);
    }

    #[test]
    fn shader_cache_is_generation_sensitive() {
        let mut cache = ShaderCache::default();
        cache.insert(TranslatedShader {
            guest_hash: 0x1234,
            stage: ShaderStage::Vertex,
            source_generation: 9,
            webgpu_source: "@vertex fn main() {}".into(),
        });
        assert!(cache.get(ShaderStage::Vertex, 0x1234, 9).is_some());
        assert!(cache.get(ShaderStage::Vertex, 0x1234, 10).is_none());
        cache.invalidate_generation(10);
        assert!(cache.is_empty());
    }

    struct TestShaderTranslator {
        calls: usize,
    }

    impl ShaderTranslator for TestShaderTranslator {
        fn translate(&mut self, stage: ShaderStage, guest_code: &[u8]) -> Result<String, String> {
            self.calls += 1;
            Ok(format!("// {stage:?} {}", guest_code.len()))
        }
    }

    #[test]
    fn shader_pipeline_translates_once_per_generation() {
        let mut pipeline = ShaderPipeline::default();
        let mut translator = TestShaderTranslator { calls: 0 };
        let first = pipeline
            .resolve(ShaderStage::Fragment, 44, 2, &[1, 2], &mut translator)
            .unwrap()
            .webgpu_source
            .clone();
        let second = pipeline
            .resolve(ShaderStage::Fragment, 44, 2, &[1, 2], &mut translator)
            .unwrap()
            .webgpu_source
            .clone();
        assert_eq!(first, second);
        assert_eq!(translator.calls, 1);
        pipeline.source_changed(3);
        pipeline
            .resolve(ShaderStage::Fragment, 44, 3, &[1, 2], &mut translator)
            .unwrap();
        assert_eq!(translator.calls, 2);
    }
}
