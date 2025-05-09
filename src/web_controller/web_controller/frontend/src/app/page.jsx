"use client";

import Controller from "@/comps/Controller";
import VelocityDisplay from "@/comps/VelocityDisplay";
import styles from "./page.module.css";
import React, { useContext } from "react";
// import RobotRender from "@/comps/RobotRender";

import { DarkmodeContext } from "@/contexts/ThemeProvider.jsx";

import {
  Divider,
  Stack,
  Typography,
  IconButton,
  Box,
  Tooltip,
} from "@mui/material";
import Viewer from "@/comps/Viewer";
import DarkModeIcon from "@mui/icons-material/DarkMode";
import LightModeIcon from "@mui/icons-material/LightMode";

export default function Home() {
  // const [position, setPosition] = useState([0, 0]);
  // const [linearVelocity, setLinearVelocity] = useState([0, 0]);
  // const wheelVectors = useRef([0, 0, 0, 0]);
  // const wheelAngles = useRef([0, 0, 0, 0]);

  return (
    <div className={styles.page}>
      <main className={styles.main}>
        <Stack direction="row" justifyContent={"space-between"}>
          <Typography variant="h4" gutterBottom>
            Hermes Control
          </Typography>
          <DarkModeSwitch />
        </Stack>
        <Divider />
        <Stack direction="row" spacing={3}>
          <Controller/>
          <Divider orientation="vertical" flexItem />
          <Viewer
            wheel_orientations={[0, 0, 0, 0]}
          />
          <VelocityDisplay/>
          {/* <Box sx = {{position : "relative"}}>
          <RobotRender 
            position={position} 
            linearVelocity={linearVelocity} 
            wheelVectors={wheelVectors} 
            wheelAngles={wheelAngles} 
            radius = {20}
            spacing = {50}
            angularVelocity = {0}
            robotAngle = {0}
          />
          </Box> */}
        </Stack>
      </main>
    </div>
  );
}

function DarkModeSwitch(props) {
  const { isDark, setIsDark } = useContext(DarkmodeContext);

  return (
    <Box sx={props.sx}>
      <Tooltip title="Toggle light/dark theme" placement="bottom">
        <IconButton onClick={() => setIsDark(!isDark)} color="secondary">
          {isDark ? <DarkModeIcon /> : <LightModeIcon />}
        </IconButton>
      </Tooltip>
    </Box>
  );
}
