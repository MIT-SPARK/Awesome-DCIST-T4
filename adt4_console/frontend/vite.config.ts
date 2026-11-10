import {defineConfig} from "vite";
import react from "@vitejs/plugin-react";

const backend = process.env.CONSOLE_BACKEND ?? "http://127.0.0.1:8090";

export default defineConfig({
  plugins: [react()],
  base: "./",
  build: {
    outDir: "../adt4_console/static",
    emptyOutDir: true,
    chunkSizeWarningLimit: 2000,
    rollupOptions: {
      output: {
        manualChunks: {three: ["three", "@react-three/fiber", "@react-three/drei"]},
      },
    },
  },
  server: {
    proxy: {"/api": {target: backend, ws: true, changeOrigin: true}},
  },
  test: {environment: "jsdom", include: ["tests/**/*.test.ts", "tests/**/*.test.tsx"]},
});
