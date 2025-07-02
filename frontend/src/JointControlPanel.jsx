import { useState, useEffect } from 'react';
import axios from 'axios';
import Alert from '@mui/material/Alert';
import { MathUtils } from 'three';

// Functional component to control the 6-axis robot using sliders to adjust the joint's angles
export default function JointControlPanel() {
  const [angles, setAngles] = useState(() => {
  const initialAngles = Array(6).fill(0);
  initialAngles[1] = -90;
  return initialAngles;
  });

  const [alertMsg, setAlertMsg] = useState('');
  const [alertSeverity, setAlertSeverity] = useState('success');

  // Update the angles when changed on slider
  const handleChange = (i, val) => {
    const updated = [...angles];
    updated[i] = parseFloat(val);
    setAngles(updated);
  };

  useEffect(() => {
    // Close alert message after 1s
    if (alertMsg) {
      const timer = setTimeout(() => setAlertMsg(''), 1000);
      return () => clearTimeout(timer);
    }
  }, [alertMsg]);

// Send a command to move the robot to the target joint angles,
// and display a status message upon successful or failed transmission.
  const sendToRobot = async () => {
    try {
      const radianAngles = angles.map(angle => MathUtils.degToRad(angle));
      const response = await axios.post(import.meta.env.VITE_API_URL + '/move', {
  angles: radianAngles
});
      setAlertMsg(response.data.message);
      setAlertSeverity('success');
      // Update slider values after robot has finnished moving
      if (response.data.angles) {
      setAngles(response.data.angles.map(angle => Math.round(MathUtils.radToDeg(Number(angle)))));
      }
      // Display error message if it occurs
    } catch (error) {
      setAlertMsg(error.response?.data?.detail || error.message);
      setAlertSeverity('error');
    }
  };

  return (
    <>
      {/* Alert message at the top of the page */}
      {alertMsg && (
        <div style={{
          position: 'fixed',
          top: 16,
          left: '80%',
          transform: 'translateX(-50%)',
          zIndex: 2000,
          minWidth: 300,
          maxWidth: 600,
        }}>
          <Alert severity={alertSeverity} onClose={() => setAlertMsg('')}>
            {alertMsg}
          </Alert>
        </div>
      )}

      {/*Control Panel */}
      <div
        style={{
          position: 'fixed',
          top: '50%',
          left: '85%',
          transform: 'translate(-50%, -50%)',
          width: 350,
          background: 'black',
          zIndex: 1200,
          padding: 24,
          boxShadow: '0 2px 8px rgba(0,0,0,0.1)',
          overflowY: 'auto',
          maxHeight: '90vh'
        }}
      >
        <h2>Joint Control Panel</h2>
        {angles.map((val, i) => (
          <div key={i}>
            <label>Joint {i + 1}</label>
            <input
              type="range"
              min={-180}
              max={180}
              value={val}
              onChange={(e) => handleChange(i, e.target.value)}
            />
            <span> {val}°</span>
          </div>
        ))}
        <button onClick={sendToRobot}>Move Robot</button>
      </div>
    </>
  );
}