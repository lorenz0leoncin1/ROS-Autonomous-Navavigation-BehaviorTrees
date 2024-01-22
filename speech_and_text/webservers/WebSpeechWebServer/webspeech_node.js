#!/usr/bin/env node

var path = require('path');
const express = require('express');
const app = express();
const http = require('http');
const server = http.createServer(app);
const { Server } = require("socket.io");
const io = new Server(server);
const rosnodejs = require('rosnodejs');
const open = require('open');


// view engine setup
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'ejs');

//Setting up public folder
app.use(express.static(__dirname + '/public'));

//ROS node comunication fragment
rosnodejs.initNode('webspeech_node')
.then(() => {
  console.log("Initializing WebSpeech ROS Node")
})

const nh = rosnodejs.nh;

// PARTE TTS (Client-Service)

const service = nh.advertiseService('cloud_tts', 'speech_and_text/Text', (req, resp) => {
  console.log("Requested sentence for TTS Service: " + req.input)
  open('http://localhost:3000/tts?text=' + req.input);
  resp.response = true
  return true;
});

app.get('/tts', (req, res) => {
  res.render('tts.ejs', {title: "WebSpeech TTS Service", lang :"en-US",  stringa: req.query.text});
});

//PARTE STT (Publisher-Subscriber)

//Gestisco il publishing su Text
const pub = nh.advertise('stt_text', 'std_msgs/String');

//Socket.io comunication with client fragment
io.on('connection', (socket) => {
  console.log('Client connected');

  socket.on('stt', (msg) => {
    console.log('I received: ' + msg);
    console.log("Sending it to ROS topic Text")
    pub.publish({ data: msg });
  });

  socket.on('disconnect', () => {
    console.log('Client disconnected');
  });
});

app.get('/stt', (req, res) => {
  res.render("stt.ejs", { title: "WebSpeech SST Service" })
});

server.listen(3000, () => {
  console.log('listening on *:3000');
});