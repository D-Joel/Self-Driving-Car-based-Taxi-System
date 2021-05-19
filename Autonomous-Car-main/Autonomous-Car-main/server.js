const express = require("express")
const bodyParser = require("body-parser")
var firebase = require('firebase');
const admin=require('firebase-admin');
var serviceAccount = require('./admin.json');
admin.initializeApp({
credential: admin.credential.cert(serviceAccount),
databaseURL: "https://uppi-174cf-default-rtdb.firebaseio.com/",
authDomain: "uppi-174cf.firebaseapp.com",
});

var db=admin.database();
var userRef=db.ref("Orders");
const app = express();

app.listen(process.env.PORT || 3000,function (){
  console.log('server is up')
  //addUser({"name":"TAXI"})
})

app.use(express.static("public"))

app.use(bodyParser.urlencoded({
  extended:true
}))

app.get("/",function(req,res){
  res.sendFile("Autonomous.html", { root: '.' })
 
})

app.post("/",function(req,res){
  var name = req.body.Name
  var phoneNumber = req.body.Number
  var pickUpDate = req.body.Pickup_Date
  var pickUpTime = req.body.Pickup_Time
  var pickUpLocation = req.body.Pickup
  var dropLocation = req.body.Drop
  userRef.push({"Name":name,"phoneNumber":phoneNumber,"pickUpDate":pickUpDate,"pickUpTime":pickUpTime,"pickUpLocation":pickUpLocation,"dropLocation":dropLocation})
  console.log("post recived",name,phoneNumber,pickUpDate,pickUpTime)
  res.sendFile("orderConfirm.html",{root: '.'})
})
