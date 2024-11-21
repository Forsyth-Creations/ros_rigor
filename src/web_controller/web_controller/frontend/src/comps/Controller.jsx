"use client";

import React, { useState, useEffect } from 'react';
import { Button, Grid, Box, Typography, Alert, Slider } from '@mui/material';
import { ArrowUpward, ArrowDownward, ArrowForward, ArrowBack, Stop } from '@mui/icons-material';
import axios from 'axios';

// API Endpoint
const API_ENDPOINT = 'http://localhost:5000';

// useMutation
import { useMutation } from "@tanstack/react-query";

const RobotController = () => {
  const [activeKey, setActiveKey] = useState(null); // Track which button is active
  const [error, setError] = useState(null);
  const [speed, setSpeed] = useState(50); // Default speed value

  const sendCommand = async (command) => {
    try {
      const response = await axios.post(`${API_ENDPOINT}/move`, command);
      console.log(response.data);
    } catch (error) {
      // Handle error
      throw new Error(error);
    }
  }

  // useMutation
  const sendCommandMutation = useMutation({
    mutationFn: sendCommand,
    onError: (error) => {
      setError(error);
    },
    onSuccess: (data) => {
      console.log(data);
      setError(null);
    }
  })

  const moveRobot = (direction) => {
    // Send command with the current speed
    console.log(`Moving robot ${direction} at speed ${speed}`);
    sendCommandMutation.mutate({ direction : direction, speed: speed });
  };

  const stopRobot = () => {
    console.log('Emergency Stop');
    setActiveKey(null); // Reset the active key (button color reset)
    sendCommandMutation.mutate({ direction: 'Stop', speed: 0 });
  };

  const handleKeyDown = (event) => {
    switch (event.key) {
      case 'ArrowUp':
      case 'w':
        setActiveKey('up');
        moveRobot('Forward');
        break;
      case 'ArrowDown':
      case 's':
        setActiveKey('down');
        moveRobot('Backward');
        break;
      case 'ArrowLeft':
      case 'a':
        setActiveKey('left');
        moveRobot('Left');
        break;
      case 'ArrowRight':
      case 'd':
        setActiveKey('right');
        moveRobot('Right');
        break;
      default:
        break;
    }
  };

  const handleKeyUp = (event) => {
    switch (event.key) {
      case 'ArrowUp':
      case 'w':
        setActiveKey(null);
        break;
      case 'ArrowDown':
      case 's':
        setActiveKey(null);
        break;
      case 'ArrowLeft':
      case 'a':
        setActiveKey(null);
        break;
      case 'ArrowRight':
      case 'd':
        setActiveKey(null);
        break;
      default:
        break;
    }
  };

  useEffect(() => {
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    // Cleanup event listeners on component unmount
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  return (
    <Box sx={{ textAlign: 'center', padding: 2 }}>
      {error && <Alert severity="error">{error.message}</Alert>}
      {!error && <Alert severity="success">No errors</Alert>}
      <Typography variant="h4" gutterBottom>
        Robot Controller
      </Typography>

      {/* Speed Control */}
      <Typography variant="body1">Speed: {speed}</Typography>
      <Slider
        value={speed}
        onChange={(e, newValue) => setSpeed(newValue)}
        aria-labelledby="speed-slider"
        min={0}
        max={100}
        valueLabelDisplay="auto"
        valueLabelFormat={(value) => `${value}%`}
        sx={{ width: 300, margin: '20px auto' }}
      />

      {/* Emergency Stop Button */}
      <Button
        variant="contained"
        color="error"
        onClick={stopRobot}
        sx={{
          width: 150,
          height: 150,
          fontSize: 24,
          marginBottom: 2,
        }}
      >
        <Stop sx={{ fontSize: 60 }} />
        <Typography variant="body1">Stop</Typography>
      </Button>

      {/* Directional Controls */}
      <Grid container spacing={2} justifyContent="center">
        {/* Row for Up */}
        <Grid item xs={12}>
          <Grid container justifyContent="center" spacing={2}>
            <Grid item>
              <Button
                variant="contained"
                onClick={() => { moveRobot('Forward'); setActiveKey('up'); }}
                sx={{
                  width: 100,
                  backgroundColor: activeKey === 'up' ? 'green' : 'primary.main',
                  '&:hover': { backgroundColor: activeKey === 'up' ? 'darkgreen' : 'primary.dark' }
                }}
              >
                <ArrowUpward sx={{ fontSize: 40 }} />
              </Button>
            </Grid>
          </Grid>
        </Grid>

        {/* Row for Left, Center (Stop), and Right */}
        <Grid item xs={12}>
          <Grid container justifyContent="center" spacing={2}>
            <Grid item>
              <Button
                variant="contained"
                onClick={() => { moveRobot('Left'); setActiveKey('left'); }}
                sx={{
                  width: 100,
                  backgroundColor: activeKey === 'left' ? 'blue' : 'primary.main',
                  '&:hover': { backgroundColor: activeKey === 'left' ? 'darkblue' : 'primary.dark' }
                }}
              >
                <ArrowBack sx={{ fontSize: 40 }} />
              </Button>
            </Grid>
            <Grid item>
              <Button
                variant="contained"
                onClick={() => { moveRobot('Right'); setActiveKey('right'); }}
                sx={{
                  width: 100,
                  backgroundColor: activeKey === 'right' ? 'red' : 'primary.main',
                  '&:hover': { backgroundColor: activeKey === 'right' ? 'darkred' : 'primary.dark' }
                }}
              >
                <ArrowForward sx={{ fontSize: 40 }} />
              </Button>
            </Grid>
          </Grid>
        </Grid>

        {/* Row for Down */}
        <Grid item xs={12}>
          <Grid container justifyContent="center" spacing={2}>
            <Grid item>
              <Button
                variant="contained"
                onClick={() => { moveRobot('Backward'); setActiveKey('down'); }}
                sx={{
                  width: 100,
                  backgroundColor: activeKey === 'down' ? 'yellow' : 'primary.main',
                  '&:hover': { backgroundColor: activeKey === 'down' ? 'orange' : 'primary.dark' }
                }}
              >
                <ArrowDownward sx={{ fontSize: 40 }} />
              </Button>
            </Grid>
          </Grid>
        </Grid>
      </Grid>

      <Typography variant="body1" mt={2}>
        Current Direction: {activeKey ? activeKey.charAt(0).toUpperCase() + activeKey.slice(1) : 'None'}
      </Typography>
    </Box>
  );
};

export default RobotController;
