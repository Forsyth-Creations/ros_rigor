import React from "react";
import { useQuery } from "@tanstack/react-query";
import axios from "axios";
import { Card, CardContent, Typography, Paper, Stack } from "@mui/material";

const getWheelData = async () => {
  const response = await axios.get("http://localhost:5000/velocity_data");
  return response.data;
};

const useGetWheelData = () => {
  return useQuery({
    queryKey: ["velocity_data"],
    queryFn: getWheelData,
    refetchInterval: 100, // Poll every 1 second
  });
};

const VelocityDisplay = () => {
  const { data, isLoading, error } = useGetWheelData();

  if (isLoading) {
    return <Typography>Loading...</Typography>;
  }

  if (error) {
    return <Typography>Error fetching data</Typography>;
  }

  return (
    <Paper elevation={3} sx={{ padding: 3 }}>
      <Typography variant="h5" gutterBottom>
        Commanded Velocity Data
      </Typography>
      <Stack direction="row" spacing={2}>
        <Card sx={{ flex: 1 }}>
          <CardContent>
            <Typography variant="h6">Linear Velocity (m/s)</Typography>
            <Typography>Linear X: {data?.linear?.x} m/s</Typography>
            <Typography>Linear Y: {data?.linear?.y} m/s</Typography>
            <Typography>Linear Z: {data?.linear?.z} m/s</Typography>
          </CardContent>
        </Card>
        <Card sx={{ flex: 1 }}>
          <CardContent>
            <Typography variant="h6">Angular Velocity (rad/s)</Typography>
            <Typography>Angular X: {data?.angular?.x} rad/s</Typography>
            <Typography>Angular Y: {data?.angular?.y} rad/s</Typography>
            <Typography>Angular Z: {data?.angular?.z} rad/s</Typography>
          </CardContent>
        </Card>
      </Stack>
    </Paper>
  );
};

export default VelocityDisplay;
