import { useState } from 'react'
import './App.css'
import JointControlPanel from './JointControlPanel'

function App() {
  const [count, setCount] = useState(0)

  return (
    <div>
      <JointControlPanel/>
    </div>
  )
}

export default App
