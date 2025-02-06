import { ReactQueryProvider } from "@/wrappers/QueryWrapper";
import ForsythTheme from "@/contexts/ThemeProvider";

import "./globals.css";

function RootLayout({ children }) {
  return (
    <html lang="en">
      <body>
        <ForsythTheme>
          <ReactQueryProvider>{children}</ReactQueryProvider>
        </ForsythTheme>
      </body>
    </html>
  );
}

export default RootLayout;
