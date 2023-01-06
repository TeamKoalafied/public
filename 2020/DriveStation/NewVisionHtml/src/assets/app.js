// Grab Aim Cam
var frontCanvas = document.getElementById("front");
var backCanvas = document.getElementById("back");
var ctx1 = frontCanvas.getContext("2d");
var ctx2 = backCanvas.getContext("2d");
// Set little green line
ctx1.fillStyle = "#04db21";
ctx1.fillRect(backCanvas.width/2, 0, 2.5, backCanvas.height);
ctx2.fillStyle = "#04db21";
ctx2.fillRect(frontCanvas.width/2, 0, 2.5, frontCanvas.height);


// 👍 documentation

// Find DOM elements
const RHardLimit = document.getElementById('RHardLimit')
const telemStatus = document.getElementById('telemetry')
const heading = document.getElementById('heading')
const shooterRPM = document.getElementById('shooterrpm')
const winchPosition = document.getElementById('winchposition')
const aimX = document.getElementById('aimx')
const winchLimit = document.getElementById('winchlimit')


// Show toast on connect
NetworkTables.addRobotConnectionListener(function(connected){
  displaytoast(`Robot Telemetry: ${connected}`, 3)
  if (connected) {
    setSquareColour(telemStatus, 'green')
  } else {
    setSquareColour(telemStatus, 'red')
  }
}, true);


/* Updaters */
// Robot heading
NetworkTables.addKeyListener('/SmartDashboard/Heading', (key, value) => {
  setText(heading, `${value.toFixed(2)}°`)
})
// Shooter target rpm
NetworkTables.addKeyListener('/SmartDashboard/Shooter Speed RPM', (key, value) => {
  setText(shooterRPM, `${value} RPM`)
})
// Shooter target rpm
NetworkTables.addKeyListener('/SmartDashboard/Winch Position Inch', (key, value) => {
  setText(winchPosition, `${value.toFixed(2)} IN`)
})
// Vision aim x
NetworkTables.addKeyListener('/pivision/tx', (key, value) => {
  setText(aimX, `${value.toFixed(2)}°`)
})
NetworkTables.addKeyListener('/pivision/tx', (key, value) => {
  if (winchLimit) {
    setSquareColour(winchLimit, 'green')
  } else {
    setSquareColour(winchLimit, 'green')
  }
})


/* UI Updates */
function setSquareColour(htmlobject, colour) {
  htmlobject.style.backgroundColor = `var(--${colour})` 
}
function setText(htmlobject, text) {
  htmlobject.innerText = text
}