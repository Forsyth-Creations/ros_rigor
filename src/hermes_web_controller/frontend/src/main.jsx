import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import MainAapp from '@/app/page.jsx'
import Layout from '@/app/layout.jsx'

createRoot(document.getElementById('root')).render(
  <StrictMode>
    <Layout>
    <MainAapp />
</Layout>   
  </StrictMode>,
)