import type { Metadata } from "next";
import localFont from "next/font/local";
import { ReactQueryProvider } from "@/wrappers/QueryWrapper";
import ForsythTheme from "@/contexts/ThemeProvider";

import "./globals.css";

const geistSans = localFont({
  src: "./fonts/GeistVF.woff",
  variable: "--font-geist-sans",
  weight: "100 900",
});
const geistMono = localFont({
  src: "./fonts/GeistMonoVF.woff",
  variable: "--font-geist-mono",
  weight: "100 900",
});

export const metadata: Metadata = {
  title: "Hermes Controller",
  description: "Written by Henry Forsyth",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <body className={`${geistSans.variable} ${geistMono.variable}`}>
        <ForsythTheme>
          <ReactQueryProvider>{children}</ReactQueryProvider>
        </ForsythTheme>
      </body>
    </html>
  );
}
