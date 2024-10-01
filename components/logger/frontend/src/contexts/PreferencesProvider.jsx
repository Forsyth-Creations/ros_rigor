"use client";
// Write me a basic context

import React, { createContext, useContext, useState } from "react";

export const PreferenceContext = createContext();

export const PreferenceProvider = ({ children }) => {
  const [theme, setTheme] = useState("light");
  const [visual, setVisual] = useState("bar");

  return (
    <PreferenceContext.Provider value={{ theme, setTheme, visual, setVisual }}>
      {children}
    </PreferenceContext.Provider>
  );
};
