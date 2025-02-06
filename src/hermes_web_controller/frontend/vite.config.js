import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import path from 'path'

const projectRootDir = path.resolve(__dirname, './src')

console.log("----------------------------------------")
console.log('Project Root Directory:', projectRootDir);
console.log("----------------------------------------")

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
    resolve: {
        alias: [
            { find: '@', replacement: projectRootDir },
        ]
    },
    server : {
        port: 3000,
    },
    

})
