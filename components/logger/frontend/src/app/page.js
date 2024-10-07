"use client";
import {
  useGetContainers,
  useGetLoggingStatus,
  startLogging,
  stopLogging,
} from "@/hooks/DockerHooks";

import Nav from "@/comps/Nav";
import {
  Box,
  Paper,
  Container,
  Typography,
  Stack,
  LinearProgress,
  Alert,
  Chip,
  Button,
} from "@mui/material";

import LoadingButton from "@mui/lab/LoadingButton";

import DockerContainer from "@/comps/Container";

export default function Home() {
  return (
    <Box sx={{ maxHeight: "100vh" }}>
      <Nav />
      <Container component={Stack} spacing={1} sx={{ mt: "6px" }}>
        <LoggingButton />
        <Containers />
      </Container>
    </Box>
  );
}

function LoggingButton() {
  const { data, isLoading, isError } = useGetLoggingStatus();

  if (isLoading) {
    return (
      <LoadingButton loading variant="contained">
        Loading
      </LoadingButton>
    );
  }

  if (isError) {
    return <Alert severity="error">Error loading logging status</Alert>;
  }

  const isLogging = Boolean(data); // Ensure data is treated as a boolean value

  function handleClick() {
    if (isLogging) {
      stopLogging();
    } else {
      startLogging();
    }
  }

  return (
    <Button
      variant="contained"
      color={isLogging ? "secondary" : "primary"}
      onClick={handleClick}
    >
      {isLogging ? "Stop Logging" : "Start Logging"}
    </Button>
  );
}

function Containers() {
  const { data, isLoading, isError } = useGetContainers();

  if (isLoading) return <LinearProgress />;

  if (isError) return <Alert severity="error">Error loading containers</Alert>;

  return (
    <Stack spacing={1}>
      {Object.entries(data).map(([key, value]) => (
        <DockerContainer entry={key} key={key} value={value} />
      ))}
    </Stack>
  );
}
