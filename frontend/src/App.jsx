import { useState } from 'react'
import reactLogo from './assets/react.svg'
import viteLogo from '/vite.svg'
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
