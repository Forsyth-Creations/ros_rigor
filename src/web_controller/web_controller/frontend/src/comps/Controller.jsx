"use client";

import React, { useState, useEffect } from "react";
import {
  Button,
  Slider,
  CircularProgress,
  Typography,
  Box,
  Stack,
  Tooltip,
  Chip
} from "@mui/material";
import axios from "axios";
import { useMutation } from "@tanstack/react-query";

// toggle
import Switch from '@mui/material/Switch';

// Check and Cross Icons
import CheckIcon from "@mui/icons-material/Check";
import CloseIcon from "@mui/icons-material/Close";

// API Endpoint
const API_ENDPOINT = "http://localhost:5000";

const RobotController = () => {
  const [error, setError] = useState(null);
  const [speed, setSpeed] = useState(50); // Default speed value
  const [direction, setDirection] = useState(0); // Default direction in radians
  const [enableLiveUpdate, setEnableLiveUpdate] = useState(false);

  // A useEffect so that, when the live update is enabled, the robot command changes, and the speed and direction are updated
  useEffect(() => {
    if (enableLiveUpdate) {
      sendCommandMutation.mutate({ speed: speed / 100, direction });
    }
  }, [enableLiveUpdate, speed, direction]);

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
    console.log(
      `Moving robot at speed ${speed} with direction ${direction.toFixed(2)} radians`,
    );
    sendCommandMutation.mutate({ speed: speed / 100, direction });
  };

  const stopRobot = () => {
    console.log("Emergency Stop");
    sendCommandMutation.mutate({ direction: 0, speed: 0 });
    // set the speed and direction to 0
    setSpeed(0);
    setDirection(0);
  };

  const setDefaultSpeed = (value) => {
    setSpeed(value);
  };

  const setDefaultDirection = (value) => {
    setDirection(value);
  };

  useEffect(() => {
    window.addEventListener("keydown", (event) => {
      if (event.key === " ") stopRobot();
    });

    return () => {
      window.removeEventListener("keydown", stopRobot);
    };
  }, [stopRobot]);

  return (
    <Stack spacing={1} alignItems="center">
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

      {/* Speed Control */}
      <Stack
        spacing={2}
        alignItems="center"
        sx={{ width: "100%", maxWidth: 400 }}
      >
        <Chip label={<Typography variant = "h6">{speed}%</Typography>} color = "primary" />
        <Slider
          value={speed}
          onChange={(e, newValue) => setSpeed(newValue)}
          aria-labelledby="speed-slider"
          min={-100}
          max={100}
          valueLabelDisplay="auto"
          valueLabelFormat={(value) => `${value}%`}
        />

        <Stack direction="row" spacing={1}>
          <Button 
            variant="outlined"
            onClick={() => setDefaultSpeed(-100)} // Set default speed to -100%
          >
            -100%
          </Button>
          <Button
            variant="outlined"
            onClick={() => setDefaultSpeed(-50)} // Set default speed to 20%
          >
            -50%
          </Button>
          {/* 0% */}
          <Button
            variant="outlined"
            onClick={() => setDefaultSpeed(0)} // Set default speed to 0%
          >
            0%
            </Button>
          <Button
            variant="outlined"
            onClick={() => setDefaultSpeed(50)} // Set default speed to 50%
          >
            50%
          </Button>
          <Button
            variant="outlined"
            onClick={() => setDefaultSpeed(100)} // Set default speed to 100%
          >
            100%
          </Button>
        </Stack>
      </Stack>

      {/* Direction Control */}
      <Stack
        spacing={1}
        alignItems="center"
        sx={{ width: "100%", maxWidth: 400 }}
      >
        <Typography variant="body1">
          Direction: {direction.toFixed(2)} radians
        </Typography>
        <Slider
          value={direction}
          onChange={(e, newValue) => setDirection(newValue)}
          aria-labelledby="direction-slider"
          min={-Math.PI}
          max={Math.PI}
          step={0.01}
          valueLabelDisplay="auto"
          valueLabelFormat={(value) => value.toFixed(2)}
        />
        <Stack direction="row" spacing={2}>
          <Button
            variant="outlined"
            onClick={() => setDefaultDirection(0)} // Set default direction to 0 radians
          >
            Straight
          </Button>
          <Button
            variant="outlined"
            onClick={() => setDefaultDirection(Math.PI / 2)} // Set default direction to 90° (π/2 radians)
          >
            Right
          </Button>
          <Button
            variant="outlined"
            onClick={() => setDefaultDirection(-Math.PI / 2)} // Set default direction to -90° (-π/2 radians)
          >
            Left
          </Button>
        </Stack>
      </Stack>

      {/* Direction Gauge */}
      <Box sx={{ position: "relative", display: "inline-flex" }}>
        <CircularProgress
          variant="determinate"
          value={((direction + Math.PI) / (2 * Math.PI)) * 100}
          size={120}
          sx={{ color: "primary.main" }}
        />
        <Box
          sx={{
            top: 0,
            left: 0,
            bottom: 0,
            right: 0,
            position: "absolute",
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
          }}
        >
          <Typography variant="h6" component="div" color="textSecondary">
            {direction.toFixed(2)} rad
          </Typography>
        </Box>
      </Box>

      {/* Emergency Stop Button */}
      <Stack direction={"row"} spacing={2} sx = {{width : "100%", maxWidth: 400}}>
      <Button
        variant="contained"
        color="error"
        onClick={stopRobot}
        fullWidth
      >
        Stop
      </Button>

      {/* Move Button */}
      <Button
        variant="contained"
        color="primary"
        onClick={moveRobot}
        fullWidth
      >
        Move Robot
      </Button>
      <Tooltip title="Enable Live Update">
        <Switch checked={enableLiveUpdate} onChange={(event) => setEnableLiveUpdate(event.target.checked)} />
      </Tooltip>
      </Stack>
    </Stack>
  );
};

export default RobotController;
