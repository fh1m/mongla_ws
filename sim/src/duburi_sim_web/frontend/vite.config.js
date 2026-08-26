import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

const labPort = process.env.DUBURI_LAB_PORT || '28765'

export default defineConfig({
  plugins: [react()],
  base: '/',
  build: {
    outDir: '../static',
    emptyOutDir: true,
  },
  publicDir: 'public',
  server: {
    proxy: {
      '/api': `http://127.0.0.1:${labPort}`,
    },
  },
})
