import { useState } from 'react'
import './App.css'
import JointControlPanel from './JointControlPanel'
import RobotViewer from './RobotViewer'

function App() {
  const [count, setCount] = useState(0)

  return (
    <div>
      <RobotViewer/>
      <JointControlPanel/>
    </div>
  )
}

export default App
