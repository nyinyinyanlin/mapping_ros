var express = require('express')
var app = express()
var path = require('path')
var bodyParser = require('body-parser')
var methodOverride = require('method-override')

app.use(express.static(path.join(__dirname,'js')));
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({extended:true}));
app.use(methodOverride());

app.get('/mapper', function(request, response, next){
    response.sendFile(path.join(__dirname,"static/mapper.html"));
});

app.get('/monitor', function(request, response, next){
    response.sendFile(path.joing(__dirname,"static/monitor.html"));
});

app.listen(3000)
