var SpeechRecognition = SpeechRecognition || webkitSpeechRecognition
var SpeechGrammarList = SpeechGrammarList || webkitSpeechGrammarList
var SpeechRecognitionEvent = SpeechRecognitionEvent || webkitSpeechRecognitionEvent


//var colors = ['aqua', 'azure', 'beige', 'bisque', 'black', 'blue', 'brown', 'chocolate', 'coral' ... ];
//var grammar = '#JSGF V1.0; grammar colors; public <color> = ' + colors.join(' | ') + ' ;'

var recognition = new SpeechRecognition();
//var speechRecognitionList = new SpeechGrammarList();

//speechRecognitionList.addFromString(grammar, 1);

//recognition.grammars = speechRecognitionList;

recognition.continuous = true;
recognition.lang = 'it-IT';
recognition.interimResults = true;
recognition.maxAlternatives = 1;

//Setting socket io
var socket = io();

var start_button = document.getElementById('start-button')
var sentences = document.getElementById('sentences')
var result = document.getElementById('result')

recognition.onresult = function (event) {
    for (var i = event.resultIndex; i < event.results.length; ++i) {
        result.textContent = event.results[i][0].transcript
        if (event.results[i].isFinal === true) {
            socket.emit('stt', result.textContent);
            console.log("Fine frase")
            result = document.createElement('p')
            sentences.appendChild(result)
        }
    }
}

recognition.onerror = function (event) {
    console.log(event.error)
}

recognition.onend = function (event) {
    startRecognition()
}


function startRecognition() {
    if (start_button.textContent == "Start listening") {
        start_button.textContent = "Stop listening"
        recognition.start();
        console.log("Started recognition")

    } else {
        start_button.textContent = "Start listening"
        recognition.stop();
        console.log("Stopped recognition")
    }
}