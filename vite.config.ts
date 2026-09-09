import { defineConfig } from 'vite'

export default defineConfig({
  base: '/OmniEmu/',
  build: {
    target: 'es2022',
    sourcemap: true,
    cssCodeSplit: false,
  },
})
