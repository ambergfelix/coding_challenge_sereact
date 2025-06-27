import React, { useState } from 'react';
import axios from 'axios';

export default function JointControlPanel() {
  const [angles, setAngles] = useState(Array(6).fill(0));

  const handleChange = (i, val) => {
    const updated = [...angles];
    updated[i] = parseFloat(val);
    setAngles(updated);
  };

  const sendToRobot = () => {
    axios.post('http://localhost:8000/move', { angles });
  };

  return (
    <div>
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
  );
}