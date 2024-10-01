import { PreferenceProvider } from "@/contexts/PreferencesProvider";
import Theme from "@/contexts/ThemeProvider";
import ClientWrapper from "@/contexts/ClientWrapper";
import { Inter } from "next/font/google";
const inter = Inter({ subsets: ["latin"] });

export const metadata = {
  title: "Logger",
  description: "A Docker/ROS Logger",
};

export default function RootLayout({ children }) {
  return (
    <html lang="en">
      <body className={inter.className} style={{ backgroundColor: "black" }}>
        <ClientWrapper>
          <Theme>
            <PreferenceProvider>{children}</PreferenceProvider>
          </Theme>
        </ClientWrapper>
      </body>
    </html>
  );
}
