"use client";

import React, { useContext } from "react";
import {
  Box,
  Toolbar,
  Typography,
  AppBar,
  IconButton,
  Tooltip,
} from "@mui/material";
import EqualizerIcon from "@mui/icons-material/Equalizer";
import FormatListBulletedIcon from "@mui/icons-material/FormatListBulleted";

// Import the PreferencesContext
import { PreferenceContext } from "@/contexts/PreferencesProvider";

export default function LoggerBar() {
  const { visual, setVisual } = useContext(PreferenceContext);

  return (
    <Box sx={{ flexGrow: 1 }}>
      <AppBar position="static">
        <Toolbar>
          <Typography variant="h6" component="div" sx={{ flexGrow: 1 }}>
            Logger
          </Typography>
          {/* Create a toggle for the visualization */}
          <Tooltip title="Toggle Visualization">
            <IconButton
              onClick={() => setVisual(visual === "bar" ? "list" : "bar")}
              color="inherit"
            >
              {visual === "bar" ? (
                <EqualizerIcon />
              ) : (
                <FormatListBulletedIcon />
              )}
            </IconButton>
          </Tooltip>
        </Toolbar>
      </AppBar>
    </Box>
  );
}
