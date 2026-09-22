use std::collections::BTreeMap;

use crate::blueprint::{GuestIsa, SystemBlueprint};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecutionMode {
    Interpreter,
    CachedInterpreter,
    WasmTranslator,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IrWidth {
    W8,
    W16,
    W32,
    W64,
    W128,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IrOp {
    Load { width: IrWidth },
    Store { width: IrWidth },
    Add { width: IrWidth },
    Sub { width: IrWidth },
    And { width: IrWidth },
    Or { width: IrWidth },
    Xor { width: IrWidth },
    Shift { width: IrWidth },
    Compare { width: IrWidth },
    Branch,
    Call,
    Return,
    Barrier,
    Vector { lanes: u8, width: IrWidth },
    Device { opcode: u16 },
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IrBlock {
    pub isa: GuestIsa,
    pub guest_pc: u64,
    pub end_pc: u64,
    pub mmu_generation: u64,
    pub ops: Vec<IrOp>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct BlockKey {
    isa: u16,
    guest_pc: u64,
}

fn isa_key(isa: GuestIsa) -> u16 {
    match isa {
        GuestIsa::DiscreteLogic => 0,
        GuestIsa::Mos6502 => 1,
        GuestIsa::Cp1610 => 2,
        GuestIsa::Z80 => 3,
        GuestIsa::W65c816 => 4,
        GuestIsa::Spc700 => 5,
        GuestIsa::HuC6280 => 6,
        GuestIsa::M68000 => 7,
        GuestIsa::Sh2 => 8,
        GuestIsa::Sh4 => 9,
        GuestIsa::MipsR3000 => 10,
        GuestIsa::MipsR4300 => 11,
        GuestIsa::MipsR5900 => 12,
        GuestIsa::Rsp => 13,
        GuestIsa::VectorUnit => 14,
        GuestIsa::ArmV3 => 15,
        GuestIsa::ArmV4 => 16,
        GuestIsa::ArmV5 => 17,
        GuestIsa::ArmV7 => 18,
        GuestIsa::ArmV8 => 19,
        GuestIsa::PowerPc32 => 20,
        GuestIsa::PowerPc64 => 21,
        GuestIsa::X86 => 22,
        GuestIsa::Spu => 23,
        GuestIsa::CustomRisc => 24,
        GuestIsa::Dsp => 25,
    }
}
#[derive(Debug, Default)]
pub struct CodeCache {
    blocks: BTreeMap<BlockKey, IrBlock>,
    epoch: u64,
}

impl CodeCache {
    pub fn epoch(&self) -> u64 {
        self.epoch
    }
    pub fn len(&self) -> usize {
        self.blocks.len()
    }
    pub fn is_empty(&self) -> bool {
        self.blocks.is_empty()
    }

    pub fn insert(&mut self, block: IrBlock) {
        let key = BlockKey {
            isa: isa_key(block.isa),
            guest_pc: block.guest_pc,
        };
        self.blocks.insert(key, block);
    }

    pub fn get(&self, isa: GuestIsa, guest_pc: u64, mmu_generation: u64) -> Option<&IrBlock> {
        let key = BlockKey {
            isa: isa_key(isa),
            guest_pc,
        };
        self.blocks
            .get(&key)
            .filter(|block| block.mmu_generation == mmu_generation)
    }

    pub fn invalidate_range(&mut self, start: u64, end: u64) {
        self.blocks
            .retain(|_, block| block.end_pc <= start || block.guest_pc >= end);
        self.epoch = self.epoch.wrapping_add(1);
    }

    pub fn invalidate_all(&mut self) {
        self.blocks.clear();
        self.epoch = self.epoch.wrapping_add(1);
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExecutionPlan {
    pub default_mode: ExecutionMode,
    pub guest_isas: Vec<GuestIsa>,
    pub needs_vector_path: bool,
    pub needs_self_modifying_code_invalidation: bool,
}
impl ExecutionPlan {
    pub fn for_system(system: &SystemBlueprint) -> Self {
        let mut guest_isas = Vec::new();
        let mut needs_vector_path = false;
        for cpu in &system.cpus {
            if !guest_isas.contains(&cpu.isa) {
                guest_isas.push(cpu.isa);
            }
            needs_vector_path |= matches!(
                cpu.isa,
                GuestIsa::VectorUnit | GuestIsa::Rsp | GuestIsa::Spu
            );
        }
        let default_mode = if system.generation <= 3 {
            ExecutionMode::Interpreter
        } else if system.generation <= 5 {
            ExecutionMode::CachedInterpreter
        } else {
            ExecutionMode::WasmTranslator
        };
        Self {
            default_mode,
            guest_isas,
            needs_vector_path,
            needs_self_modifying_code_invalidation: system.generation >= 5,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockExit {
    Fallthrough(u64),
    Branch(u64),
    Yield,
    Halt,
}

pub trait BlockTranslator {
    fn translate(
        &mut self,
        isa: GuestIsa,
        guest_pc: u64,
        mmu_generation: u64,
    ) -> Result<IrBlock, String>;
}

pub trait BlockRunner<Context> {
    fn run(&mut self, context: &mut Context, block: &IrBlock) -> Result<BlockExit, String>;
}

#[derive(Debug)]
pub struct ExecutionEngine {
    mode: ExecutionMode,
    cache: CodeCache,
}

impl ExecutionEngine {
    pub fn new(mode: ExecutionMode) -> Self {
        Self {
            mode,
            cache: CodeCache::default(),
        }
    }

    pub fn mode(&self) -> ExecutionMode {
        self.mode
    }

    pub fn cache(&self) -> &CodeCache {
        &self.cache
    }

    pub fn invalidate_range(&mut self, start: u64, end: u64) {
        self.cache.invalidate_range(start, end);
    }

    pub fn invalidate_all(&mut self) {
        self.cache.invalidate_all();
    }

    pub fn run_block<Context, Translator, Runner>(
        &mut self,
        isa: GuestIsa,
        guest_pc: u64,
        mmu_generation: u64,
        translator: &mut Translator,
        runner: &mut Runner,
        context: &mut Context,
    ) -> Result<BlockExit, String>
    where
        Translator: BlockTranslator,
        Runner: BlockRunner<Context>,
    {
        if self.mode == ExecutionMode::Interpreter {
            let block = translator.translate(isa, guest_pc, mmu_generation)?;
            Self::validate_block(&block, isa, guest_pc, mmu_generation)?;
            return runner.run(context, &block);
        }

        if self.cache.get(isa, guest_pc, mmu_generation).is_none() {
            let block = translator.translate(isa, guest_pc, mmu_generation)?;
            Self::validate_block(&block, isa, guest_pc, mmu_generation)?;
            self.cache.insert(block);
        }
        let block = self
            .cache
            .get(isa, guest_pc, mmu_generation)
            .ok_or_else(|| "translated block disappeared from the execution cache".to_string())?;
        runner.run(context, block)
    }

    fn validate_block(
        block: &IrBlock,
        isa: GuestIsa,
        guest_pc: u64,
        mmu_generation: u64,
    ) -> Result<(), String> {
        if block.isa != isa || block.guest_pc != guest_pc {
            return Err("translator returned a block for the wrong guest address".into());
        }
        if block.mmu_generation != mmu_generation {
            return Err("translator returned a block for the wrong MMU generation".into());
        }
        if block.end_pc <= block.guest_pc {
            return Err("translated block must advance the guest program counter".into());
        }
        if block.ops.is_empty() {
            return Err("translated block contains no operations".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blueprint::blueprint;
    use crate::platform::PlatformId;

    #[test]
    fn cache_is_keyed_by_isa_pc_and_mmu_generation() {
        let mut cache = CodeCache::default();
        cache.insert(IrBlock {
            isa: GuestIsa::MipsR5900,
            guest_pc: 0x1000,
            end_pc: 0x1040,
            mmu_generation: 7,
            ops: vec![IrOp::Add {
                width: IrWidth::W64,
            }],
        });
        assert!(cache.get(GuestIsa::MipsR5900, 0x1000, 7).is_some());
        assert!(cache.get(GuestIsa::MipsR5900, 0x1000, 8).is_none());
        assert!(cache.get(GuestIsa::Sh4, 0x1000, 7).is_none());
    }

    #[test]
    fn range_invalidation_drops_overlapping_blocks_only() {
        let mut cache = CodeCache::default();
        for pc in [0x1000, 0x2000, 0x3000] {
            cache.insert(IrBlock {
                isa: GuestIsa::ArmV8,
                guest_pc: pc,
                end_pc: pc + 0x40,
                mmu_generation: 1,
                ops: vec![IrOp::Barrier],
            });
        }
        cache.invalidate_range(0x1ff0, 0x2100);
        assert_eq!(cache.len(), 2);
        assert!(cache.get(GuestIsa::ArmV8, 0x2000, 1).is_none());
    }

    #[test]
    fn sixth_through_eighth_generation_use_translation_plan() {
        let switch = blueprint(PlatformId::Switch).unwrap();
        let plan = ExecutionPlan::for_system(&switch);
        assert_eq!(plan.default_mode, ExecutionMode::WasmTranslator);
        assert!(plan.guest_isas.contains(&GuestIsa::ArmV8));
        let ps3 = blueprint(PlatformId::PlayStation3).unwrap();
        assert!(ExecutionPlan::for_system(&ps3).needs_vector_path);
    }

    struct TestTranslator {
        translations: usize,
    }

    impl BlockTranslator for TestTranslator {
        fn translate(
            &mut self,
            isa: GuestIsa,
            guest_pc: u64,
            mmu_generation: u64,
        ) -> Result<IrBlock, String> {
            self.translations += 1;
            Ok(IrBlock {
                isa,
                guest_pc,
                end_pc: guest_pc + 4,
                mmu_generation,
                ops: vec![IrOp::Barrier],
            })
        }
    }

    struct TestRunner;

    impl BlockRunner<u64> for TestRunner {
        fn run(&mut self, context: &mut u64, block: &IrBlock) -> Result<BlockExit, String> {
            *context += block.ops.len() as u64;
            Ok(BlockExit::Fallthrough(block.end_pc))
        }
    }

    #[test]
    fn cached_execution_translates_once_until_invalidated() {
        let mut engine = ExecutionEngine::new(ExecutionMode::WasmTranslator);
        let mut translator = TestTranslator { translations: 0 };
        let mut runner = TestRunner;
        let mut context = 0;
        for _ in 0..2 {
            assert_eq!(
                engine
                    .run_block(
                        GuestIsa::ArmV8,
                        0x4000,
                        7,
                        &mut translator,
                        &mut runner,
                        &mut context,
                    )
                    .unwrap(),
                BlockExit::Fallthrough(0x4004)
            );
        }
        assert_eq!(translator.translations, 1);
        assert_eq!(context, 2);
        engine.invalidate_range(0x4000, 0x4004);
        engine
            .run_block(
                GuestIsa::ArmV8,
                0x4000,
                7,
                &mut translator,
                &mut runner,
                &mut context,
            )
            .unwrap();
        assert_eq!(translator.translations, 2);
    }
}
