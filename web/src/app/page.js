"use client";

import React, { useState, useRef  } from "react";
import {
  Box,
  Button,
  Typography,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Paper,
  TablePagination,
  TextField,
  Stack,
  Alert,
} from "@mui/material";

// x-graphs
import { BarChart } from "@mui/x-charts/BarChart";
import { LineChart } from "@mui/x-charts/LineChart";

// --------------------- Constants -------------------------

const HIGH_SPEED = 10;
const LOW_SPEED = 5;

const SENSOR_HIGH_PERIOD = 40;
const SENSOR_LOW_PERIOD = 10;

// ---------------------------------------------------------

const RobotSim = () => {
  const [robotPosition, setRobotPosition] = useState(0);
  const [isRunning, setIsRunning] = useState(false);
  const [isComplete, setIsComplete] = useState(false);
  const [simulationData, setSimulationData] = useState([]); // Track simulations
  const [simulationId, setSimulationId] = useState(1);
  const [page, setPage] = useState(0); // Current page state
  const [rowsPerPage, setRowsPerPage] = useState(5); // Number of rows per page
  const [perceptionRange, setPerceptionRange] = useState(50); // Perception range of the robot
  const [staticBlockPosition, setStaticPosition] = useState(300);
  const [simTime, setSimTime] = useState(0);
  const [scanningPeriod, setScanningPeriod] = useState(30); // Scanning frequency of the robot
  const [sensorActive, setSensorActive] = useState(false); // Sensor state
  const [crashedCount, setCrashedCount] = useState(0); // Number of crashes
  const [avoidCount, setAvoidCount] = useState(0); // Number of avoided crashes
  const [simulationCount, setSimulationCount] = useState(10); // Number of simulations
  const [epochs, setEpochs] = useState(10); // Number of epochs
  const [currentEpoch, setCurrentEpoch] = useState(1); // Current epoch
  const [historicData, setHistoricData] = useState([]); // Historic simulation data
  const [simStartTime, setSimStartTime] = useState(0); // Simulation start time
  const cancelRef = useRef(false);
  const explorationRate = useRef(0.2);
  const blockSize = 50; // Size of the blocks

  const [crashed, setCrashed] = useState(false);
  const [avoided, setAvoided] = useState(false);

  function getRandomAction() {
    // pick a number 1 through 4
    return Math.floor(Math.random() * 4);
  }

  function getAction(index) {
    let num_index = parseInt(index);
    switch (num_index) {
      case 0:
        return [HIGH_SPEED, SENSOR_HIGH_PERIOD];
      case 1:
        return [HIGH_SPEED, SENSOR_LOW_PERIOD];
      case 2:
        return [LOW_SPEED, SENSOR_HIGH_PERIOD];
      case 3:
        return [LOW_SPEED, SENSOR_LOW_PERIOD];
    }
  }

  function getActionName(index) {
    let num_index = parseInt(index);
    switch (num_index) {
      case 0:
        return "High Speed, High Period";
      case 1:
        return "High Speed, Low Period";
      case 2:
        return "Low Speed, High Period";
      case 3:
        return "Low Speed, Low Period";
    }
  }

  // ------------------------------ Q State Learning Section -------------------------------
  const [QTable, setQTable] = useState({});
  // Initialize Q-Table

  // Q-Learning Hyperparameters
  const learningRate = 0.1; // How much to learn from new experience
  const discountFactor = 0.9; // Importance of future rewards

  const chooseAction = (state, QTable2) => {
    // console.log(QTable);
    if (Math.random() < explorationRate.current || !QTable2[state]) {
      // Explore: choose a random action
      console.log(`Exploring with rate: ${explorationRate.current}`);
      return getRandomAction();
    } else {
      // Exploit: choose the best action
      console.log(`Exploiting with rate: ${explorationRate.current}`);
      const stateActions = QTable2[state] || {};
      return Object.keys(stateActions).reduce((best, action) => 
        stateActions[action] > (stateActions[best] || 0) ? action : best,
      "",);
    }
  };

  const updateQValue = (QTable2, state, action, reward, nextState) => {
    // console.log(`Updating Q-Value for state: ${state}, action: ${action}, reward: ${reward}, nextState: ${nextState}`);
    // console.log(QTable2);
    const currentQValue = (QTable2[state] && QTable2[state][action]) || 0;
    const maxNextQValue = Math.max(...Object.values(QTable2[nextState] || { 0: 0 }));
    const newQValue = currentQValue + learningRate * (reward + discountFactor * maxNextQValue - currentQValue);
    // console.log(`New Q-Value: ${newQValue}`);
    QTable2[state] = { ...QTable2[state], [action]: newQValue };
    setQTable(QTable2);
  };

  const handleRunBestAction = async () => {

    let count_crash = 0;
    let count_avoid = 0;

    let sim_id = simulationId + 1;
    setSimulationId(sim_id);

    // pull the best action from the Q-Table
    // Set teh exploration rate to 0
    explorationRate.current = 0;
    let currentState = [0, 0];
    let action = chooseAction(currentState, QTable);

    // console.log(`Best Action: ${action}`);

    let [speed, period] = getAction(action);

    let metadata = {
      "speed" : speed,
      "scanningPeriod" : period,
      epoch: 0,
    };

    // Run it for the number of episodes

    for (let i = 0; i < simulationCount; i++) {
      sim_id++;
      setSimulationId(sim_id);

      let results = await runSingleSimulation(speed, period);
      await recordSimulation(results, sim_id, metadata);

      if (results) {
        count_crash++;
        setCrashed(true);
        setAvoided(false);
      } else {
        count_avoid++;
        setAvoided(true);
        setCrashed(false);
      }

      if (cancelRef.current) {
        setIsRunning(false);
        return;
      }
    }

    // add to the historic data
    setHistoricData((prevData) => [
      ...prevData,
      { epoch: currentEpoch + 1, crashed: count_crash, avoid: count_avoid },
    ]);

    setIsRunning(false); // Stop the simulation
    setIsComplete(true); // Mark the simulation

    // iterate the epoch
    setCurrentEpoch(currentEpoch + 1);
  }


  // ---------------------------------------------------------------------------------------

  const handleStart = async () => {
    cancelRef.current = false; // Reset cancel flag
    setSimulationData([]); // Clear previous simulation data
    setCrashedCount(0); // Reset the crash count
    setAvoidCount(0); // Reset the avoid count
    setIsRunning(true); // Start the simulation
    setHistoricData([]); // Clear the historic data
    setSimStartTime(Date.now()); // Record the start time
    setIsComplete(false); // Reset the complete flag
    let starting_explore_rate = 1.0;
    explorationRate.current = starting_explore_rate; // Reset the exploration rate
    setQTable({});


    let sim_id = 0;
    let QTable2 = {};

    for (let epoch = 1; epoch <= epochs; epoch++) {
      setCurrentEpoch(epoch);
      let crashedCount2 = 0;
      let avoidCount2 = 0;
      for (let i = 0; i < simulationCount; i++) {
        // Run the simulation for the specified number of times

        sim_id++;
        setSimulationId(sim_id);

        let currentState = [0, 0];
        
        let action = chooseAction(currentState, QTable2);
        let [speed, period] = getAction(action);

        let metadata = {
          "speed" : speed,
          "scanningPeriod" : period,
          epoch,
        };

        // Get the action to take

        let results = await runSingleSimulation(speed, period);
        await recordSimulation(results, sim_id, metadata);

        if (results) {
          crashedCount2++;
          setCrashed(true);
          setAvoided(false);
        }
        else{
          avoidCount2++;
          setAvoided(true);
          setCrashed(false);
        }

        if (cancelRef.current) {
          setIsRunning(false);
          return;
        }

        const reward = results ? -10 : 10;

        const nextState = [speed, period];

        updateQValue(QTable2, currentState, action, reward, nextState);


      }
      // After each epoch, update the historic data
      // This data should be in the format that a line plot can understand
      setHistoricData((prevData) => [
        ...prevData,
        { epoch, crashed: crashedCount2, avoid: avoidCount2 },
      ]);

      // lower the exploration rate proportionally to the epoch
      explorationRate.current = explorationRate.current - (starting_explore_rate / epochs);
    }

    setIsRunning(false); // Stop the simulation
    setIsComplete(true); // Mark the simulation as complete
  };

  const runSingleSimulation = (speed, period) => {
    return new Promise((resolve) => {
      setRobotPosition(0);
      setSimTime(0);
      setSensorActive(false);
      let randomPosition = 300 + Math.floor(Math.random() * 50) * 10; // Randomize the static block position, ensuring it's a multiple of 10
      setStaticPosition(randomPosition);

      let position = 0;
      let simTime2 = 0;
      let sensorActive2 = false;

      const moveRobot = () => {
        // Check for collision before updating position
        if (position + blockSize >= randomPosition) {
          resolve(true);
          return;
        }

        // Stop if the static block is within range of perception
        if (
          sensorActive2 &&
          Math.abs(position - randomPosition) <= perceptionRange * 2
        ) {
          resolve(false);
          return;
        }

        position += speed; // Update the position manually
        setRobotPosition(position); // Update state with new position

        // Toggle the sensor state if the scanning period is reached
        // setSensorActive((prev) => simTime % scanningPeriod === 0 ? !prev : prev);
        if (simTime2 > 0 && simTime2 % period === 0) {
          sensorActive2 = !sensorActive2;
          setSensorActive(sensorActive2);
        }
        // one tick after the sensor is active, the sensor is no longer active
        if (sensorActive2 && simTime2 % period === 1) {
          sensorActive2 = false;
          setSensorActive(sensorActive2);
        }

        // Update the simulation time
        setSimTime((prevTime) => prevTime + 1);
        simTime2 = simTime2 + 1;

        requestAnimationFrame(moveRobot);
      };

      requestAnimationFrame(moveRobot);
    });
  };

  const handleStop = () => {
    cancelRef.current = true;
  };

  const recordSimulation = async (crashed, simulationId, metadata) => {
    setSimulationData((prevData) => [
      ...prevData, // Correctly appending the new simulation result
      { id: simulationId, simulation: simulationId, crashed, metadata },
    ]);
    setSimulationId((prevId) => prevId + 1); // Increment simulation ID

    // Update the bar chart data
    if (crashed) {
      setCrashedCount((prev) => prev + 1);
    } else {
      setAvoidCount((prev) => prev + 1);
    }
  };

  const handleChangePage = (event, newPage) => {
    setPage(newPage); // Change to the new page
  };

  const handleChangeRowsPerPage = (event) => {
    setRowsPerPage(parseInt(event.target.value, 10)); // Set the new rows per page
    setPage(0); // Reset to the first page
  };

  // Get the data to display on the current page
  const paginatedData = simulationData.slice(
    page * rowsPerPage,
    page * rowsPerPage + rowsPerPage,
  );

  return (
    <Box sx={{ p: 2 }}>
      <Typography variant="h4" align="center" gutterBottom>
        Basic Robot Crash Simulation
      </Typography>
      {isRunning && (
        <Alert severity="info">
          Running simulation. Epoch: {currentEpoch} and Sim ID: {simulationId}. Time running {Math.floor((Date.now() - simStartTime) / 1000)} seconds
        </Alert>
      )}
      {crashed && (<Alert severity="error">Robot Crashed!</Alert>)}
      {avoided && (<Alert severity="success">Robot Avoided Crash!</Alert>)}
      <Box
        sx={{
          position: "relative",
          width: "100%",
          height: "200px",
          border: "1px solid #ccc",
          overflow: "hidden",
          backgroundColor: "#f9f9f9",
        }}
      >
        {/* Robot Block */}
        <Box
          sx={{
            position: "absolute",
            top: "80px",
            left: `${robotPosition}px`,
            width: `${blockSize}px`,
            height: `${blockSize}px`,
            backgroundColor: "blue",
            borderRadius: "50%",
            boxShadow: "0 0 10px 5px rgba(0, 0, 255, 0.5)", // Detection ring
          }}
        >
          {/* Draw a ring around the robot */}
          {sensorActive && (
            <Box
              sx={{
                position: "absolute",
                top: "-50px",
                left: "-50px",
                width: `${blockSize + perceptionRange * 2}px`,
                height: `${blockSize + perceptionRange * 2}px`,
                borderRadius: "50%",
                border: "1px solid rgba(0, 0, 0, 0.5)",
                boxShadow: "0 0 10px 5px rgba(0, 0, 0, 0.5)",
                pointerEvents: "none", // Disable pointer events
              }}
            ></Box>
          )}
        </Box>

        {/* Static Block */}
        <Box
          sx={{
            position: "absolute",
            top: "80px",
            left: `${staticBlockPosition}px`,
            width: `${blockSize}px`,
            height: `${blockSize}px`,
            backgroundColor: "red",
          }}
        ></Box>

        {/* Sim time in the bottom right */}
        <Box
          sx={{
            position: "absolute",
            bottom: 0,
            right: 0,
            padding: 1,
            backgroundColor: "rgba(0, 0, 0, 0.5)",
            color: "white",
          }}
        >
          {`Time: ${simTime}`} {`Explr: ${Math.round( explorationRate.current * 100) / 100}`}
        </Box>
      </Box>
      <Stack
        direction="row"
        spacing={2}
        sx={{ m: "10px", width: "100%" }}
        justifyContent={"center"}
      >
        <Button
          variant="contained"
          color="primary"
          onClick={handleStart}
          disabled={isRunning}
        >
          Start
        </Button>
        <Button
          variant="contained"
          color="secondary"
          onClick={handleStop}
          disabled={!isRunning}
        >
          Stop
        </Button>
        <TextField
          label="Episodes Per Epoch"
          type="number"
          value={simulationCount}
          onChange={(e) => setSimulationCount(parseInt(e.target.value, 10))}
          sx={{ width: "200px" }}
        />
        <TextField
          label="Epochs"
          type="number"
          value={epochs}
          onChange={(e) => setEpochs(parseInt(e.target.value, 10))}
          sx={{ width: "100px" }}
        />
        <Button variant="contained" color="primary" disabled = {!isComplete} onClick = {handleRunBestAction}>
          Run Best Action based on Q-Table
        </Button>
      </Stack>
      {historicData && <Stack direction = "row" spacing = {2} sx={{ width: "100%" }}>
        <LineChart
          xAxis={[{ data: historicData && historicData.map((d) => d?.epoch), scaleType: "band" }]}
          series={[
            {
              data: historicData && historicData.map((d) => d?.crashed),
              color: "red",
            },
            {
              data: historicData && historicData.map((d) => d?.avoid),
              color: "green",
            },
          ]}
          margin={{ top: 10, bottom: 30, left: 40, right: 10 }}
          height={300}
        />
        {/* Table to show the Q table */}
        <TableContainer
          component={Paper}
          sx={{ mt: 2, width: "50vw" }}
        >
          <Table>
            <TableHead>
              <TableRow>
                <TableCell>State</TableCell>
                <TableCell>Action</TableCell>
                <TableCell>Q-Value</TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
              {Object.keys(QTable).map((state) => (
                Object.keys(QTable[state]).map((action) => (
                  <TableRow key={`${state}-${action}`}>
                    <TableCell>{state}</TableCell>
                    <TableCell>{getActionName(action)}</TableCell>
                    <TableCell>{QTable[state][action]}</TableCell>
                  </TableRow>
                ))
              ))}
            </TableBody>
          </Table>
        </TableContainer>
      </Stack>}
      <Stack direction="row" spacing={2}>
        <Box>
          <Typography variant="h6">Simulation Results</Typography>
          <TableContainer component={Paper} sx={{ mt: 2, width: "50vw" }}>
            <Table>
              <TableHead>
                <TableRow>
                  <TableCell>Simulation</TableCell>
                  <TableCell>Crashed</TableCell>
                  <TableCell>Epoch</TableCell>
                  <TableCell>Speed</TableCell>
                  <TableCell>Scanning Period</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {paginatedData.map((row) => (
                  <TableRow key={row.id}>
                    <TableCell>{row.simulation}</TableCell>
                    <TableCell>{row.crashed ? "Yes" : "No"}</TableCell>
                    <TableCell>{row.metadata.epoch}</TableCell>
                    <TableCell>{row.metadata.speed}</TableCell>
                    <TableCell>{row.metadata.scanningPeriod}</TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
            {/* Pagination */}
            <TablePagination
              rowsPerPageOptions={[5, 10, 25]}
              component="div"
              count={simulationData.length}
              rowsPerPage={rowsPerPage}
              page={page}
              onPageChange={handleChangePage}
              onRowsPerPageChange={handleChangeRowsPerPage}
            />
          </TableContainer>
        </Box>
        <Box sx={{ width: "100%" }}>
          <BarChart
            series={[
              { data: [crashedCount], color: "red" },
              { data: [avoidCount], color: "green" },
            ]}
            height={290}
            xAxis={[{ data: ["Current Results"], scaleType: "band" }]}
            margin={{ top: 10, bottom: 30, left: 40, right: 10 }}
          />
        </Box>
      </Stack>
    </Box>
  );
};

export default RobotSim;
