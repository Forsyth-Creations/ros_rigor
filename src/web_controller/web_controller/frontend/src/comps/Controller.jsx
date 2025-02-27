"use client";

import React, { useState, useEffect } from "react";
import {
  Slider,
  Typography,
  Box,
  Stack,
  Tooltip,
  Chip,
  Divider,
  IconButton
} from "@mui/material";
import axios from "axios";
import { useMutation } from "@tanstack/react-query";

// stop symbol
import StopIcon from "@mui/icons-material/Stop";

// move symbol
import SendIcon from '@mui/icons-material/Send';
// toggle
import Switch from "@mui/material/Switch";

// Check and Cross Icons
import CheckIcon from "@mui/icons-material/Check";
import CloseIcon from "@mui/icons-material/Close";

// API Endpoint
const API_ENDPOINT = "http://localhost:5000";

const RobotController = ({  }) => {
  const [error, setError] = useState(null);
  const [vel_x, setVel_x] = useState(0);
  const [vel_y, setVel_y] = useState(0);
  const [angVel, setAngVel] = useState(0); // Default direction in radians
  const [enableLiveUpdate, setEnableLiveUpdate] = useState(false);

  // A useEffect so that, when the live update is enabled, the robot command changes, and the speed and direction are updated
  useEffect(() => {
    if (enableLiveUpdate) {
      sendCommandMutation.mutate({
        vel_x: vel_x / 10,
        vel_y: vel_y / 10,
        ang_vel: angVel,
      });
    }
  }, [enableLiveUpdate, angVel, vel_x, vel_y]);

  const sendCommand = async (command) => {
    try {
      const response = await axios.post(`${API_ENDPOINT}/move`, command);
      console.log(response.data);
    } catch (error) {
      // Handle error
      throw new Error(error);
    }
  };

  // useMutation
  const sendCommandMutation = useMutation({
    mutationFn: sendCommand,
    onError: (error) => {
      setError(error);
    },
    onSuccess: (data) => {
      console.log(data);
      setError(null);
    },
  });

  const moveRobot = () => {
    sendCommandMutation.mutate({ vel_x : vel_x / 10, vel_y : vel_y / 10, ang_vel: angVel });
  };

  const stopRobot = () => {
    console.log("Emergency Stop");
    sendCommandMutation.mutate({ vel_x: 0, vel_y: 0, ang_vel: 0 });
    // set the speed and direction to 0
    setVel_x(0);
    setVel_y(0);
    setAngVel(0);
  };


  // useEffect(() => {
  //   window.addEventListener("keydown", (event) => {
  //     if (event.key === " ") stopRobot();
  //   });

  //   return () => {
  //     window.removeEventListener("keydown", stopRobot);
  //   };
  // }, [stopRobot]);

  return (
    <Stack spacing={1} alignItems="flex-start">
      {/* Title */}
      <Stack direction="row" alignItems="center" spacing={2}>
        <Box>
          <Typography variant="h5" gutterBottom>
            Robot Controller
          </Typography>
        </Box>
        <Box>
          <Tooltip title="No errors">
            {!error && <CheckIcon color="success" />}
          </Tooltip>
          <Tooltip title={error ? error.message : ""}>
            {error && <CloseIcon color="error" />}
          </Tooltip>
        </Box>
      </Stack>

      <Divider sx={{ width: "100%", maxWidth: 400 }} />
      {/* Speed Control */}
      <Stack
        spacing={2}
        alignItems="flex-start"
        sx={{ width: "100%", maxWidth: 400 }}
      >
        <Chip
          label={
            <Typography variant="h6">Linear Velocity {vel_x} m/s, {vel_y} m/s</Typography>
          }
          color="primary"
        />

        {/* X Axis */}

        <Stack direction="row" sx={{ width: "100%" }} alignItems={"center"}>
          <Typography sx={{ m: 2 }}>X</Typography>
          <Slider
            value={vel_x}
            onChange={(e, newValue) => setVel_x(newValue)}
            aria-labelledby="speed-slider"
            min={-10}
            max={10}
            valueLabelDisplay="auto"
            valueLabelFormat={(value) => `${value} m/s`}
          />
        </Stack>

        {/* Y Axis */}
        <Stack direction="row" sx={{ width: "100%" }} alignItems={"center"}>
          <Typography sx={{ m: 2 }}>Y</Typography>
          <Slider
            value={vel_y}
            onChange={(e, newValue) => setVel_y(newValue)}
            aria-labelledby="speed-slider"
            min={-10}
            max={10}
            valueLabelDisplay="auto"
            valueLabelFormat={(value) => `${value} m/s`}
          />
        </Stack>
      </Stack>

      {/* Direction Control */}
      <Stack
        spacing={1}
        alignItems="flex-start"
        sx={{ width: "100%", maxWidth: 400 }}
      >
        <Chip
          label={
            <Typography variant="h6">
              Angular Velocity: {angVel.toFixed(2)} radians/s
            </Typography>
          }
          color="primary"
        />
        <Stack direction="row" sx={{ width: "100%" }} alignItems={"center"}>
          <Typography sx={{ m: 2 }}>w</Typography>
          <Slider
            value={angVel}
            onChange={(e, newValue) => setAngVel(newValue)}
            aria-labelledby="direction-slider"
            min={-Math.PI}
            max={Math.PI}
            step={0.01}
            valueLabelDisplay="auto"
            valueLabelFormat={(value) => value.toFixed(2)}
          />
        </Stack>
      </Stack>

      {/* Emergency Stop Button */}
      <Stack
        direction={"row"}
        spacing={2}
        sx={{ width: "100%", maxWidth: 400 }}
      >
        <Tooltip title="Emergency Stop">
          <IconButton variant="contained" color="error" onClick={stopRobot} fullWidth>
            <StopIcon />
          </IconButton>
        </Tooltip>

        {/* Move Button */}
        <Tooltip title="Move Robot">
        <IconButton
          variant="contained"
          color="primary"
          onClick={moveRobot}
          fullWidth
        >
          <SendIcon/>
        </IconButton>
        </Tooltip>

        <Tooltip title="Enable Live Update">
          <Switch
            checked={enableLiveUpdate}
            onChange={(event) => setEnableLiveUpdate(event.target.checked)}
          />
        </Tooltip>
      </Stack>
    </Stack>
  );
};

export default RobotController;
