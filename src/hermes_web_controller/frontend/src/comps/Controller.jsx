"use client";

import React, { useState, useEffect } from "react";
import {
  Button,
  Slider,
  CircularProgress,
  Typography,
  Alert,
  Box,
  Stack,
} from "@mui/material";
import axios from "axios";
import { useMutation } from "@tanstack/react-query";

// API Endpoint
const API_ENDPOINT = "http://localhost:5000";

const RobotController = () => {
  const [error, setError] = useState(null);
  const [speed, setSpeed] = useState(50); // Default speed value
  const [direction, setDirection] = useState(0); // Default direction in radians

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
      {/* Alert Section */}
      {error && <Alert severity="error">{error.message}</Alert>}
      {!error && <Alert severity="success">No errors</Alert>}

      {/* Title */}
      <Typography variant="h4" gutterBottom>
        Robot Controller
      </Typography>

      {/* Speed Control */}
      <Stack
        spacing={2}
        alignItems="center"
        sx={{ width: "100%", maxWidth: 400 }}
      >
        <Typography variant="body1">Speed: {speed}</Typography>
        <Slider
          value={speed}
          onChange={(e, newValue) => setSpeed(newValue)}
          aria-labelledby="speed-slider"
          min={0}
          max={100}
          valueLabelDisplay="auto"
          valueLabelFormat={(value) => `${value}%`}
        />
      </Stack>

      {/* Direction Control */}
      <Stack
        spacing={2}
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
      <Button
        variant="contained"
        color="error"
        onClick={stopRobot}
        sx={{
          width: 150,
          height: 150,
          fontSize: 24,
        }}
      >
        Stop
      </Button>

      {/* Move Button */}
      <Button
        variant="contained"
        color="primary"
        onClick={moveRobot}
        sx={{ width: "auto" }}
      >
        Move Robot
      </Button>
    </Stack>
  );
};

export default RobotController;
