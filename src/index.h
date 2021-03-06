extern const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta name="file" content="index.html">
    <meta name="author" content="Joe Belson 20210829">
    <meta name="what" content="esp8266-12 html for WiFi/OLED connected airpressure balancer for the house">
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=480, initial-scale=1.0" />
    <title id="title">Air Pressure Balancer</title>
  </head>
  <style type="text/css">
    body {color:rgb(0, 0, 0); text-align: center; background-color: #ffffff;  font-family: Calibri, Myriad; font-weight: bold; }
    table {width: 315px;border: 1px solid black;text-align: center; }

    caption {background-color: #61f035;border: solid;border-width: 1px;font-size:x-large;}
    th.empty{background-color: rgb(255, 255, 255); border-bottom: 0px; height: 23px;}
    th.Temperature{background-color: rgb(248, 117, 117);}
    th.Pressure{background-color: rgb(255, 255, 00);}
    th.Humidity{background-color: rgb(113, 213, 218);}
    th.hot{background-color:rgba(252, 90, 90, 0.938);}

    tr:nth-child(even) {background-color: #D6EEEE;}
    tr:nth-child(odd) { background-color: rgb(218, 218, 218);}

    table .settings{background-color: rgb(138, 247, 134);}
    th.cold{background-color: rgb(111, 183, 243);}

  </style>

  <body style="text-align: right;">
    <input id="bt_help" type="button" onclick="f_help()" value="Help"/>
  <hr>
    <table class="valuesTable">
      <caption id="caption">House Internal Air Pressure</caption>
                <th class="empty"></th>
                <th class="Temperature">Temperature</th>
                <th class="Pressure">Pressure</th>
                <th class="Humidity">Humidity</th>
        <tbody>
            <tr id="outside">
                <td>OutSide</td>
                <td id="oat">inop</td>
                <td id="oap">inop</td>
                <td id="oah">inop</td>
            </tr>
            <tr id="inside">
                <td >Inside</td>
                <td id="iat">inop</td>
                <td class="iap" id="iap">inop</td>
                <td class="iah" id="iah">inop</td>
            </tr>
        </tbody>
    </table>
    <table>
      <tbody>
        <tr class="settings"><td>Damper Degrees</td><td>Draft Fan%</td><td>Leak</td></tr>
        <tr><td id="draftDamperPos">inop</td><td id="draftFanPct">0</td><td id="waterSensor">NO</td></tr>  
      </tbody>
    </table>
<table>
  <tbody><caption>Water Usage</caption>
    <th class="cold">Main</th><th class="cold">Gallon this Month</th><th class="cold">Gallon Today</th><th class="cold">GPM</th>
    <tr><td></td><td id="mainGallonsTotal">inop</td><td id="mainGallonsToday">inop</td><td id="mainFlowGPM">inop</td></tr>

    <th class="hot">Hot</th><th class="hot">Gallon this Month</th><th class="hot">Gallon Today</th><th class="hot">GPH</th>
    <tr><td></td><td id="hotGallonsTotal">inop</td><td id="hotGallonsToday">inop</td><td id="hotFlowGPM">inop</td></tr>

    <tr></tr>
  </tbody>
</table>
    <p></p>
    <p id="msg" name="msg" style="visibility: hidden;"></p>
    <p id="msg1" name="msg" style="visibility: hidden;"></p>
  </body>
  <script language = "javascript" type = "text/javascript">

    var lastUpdated = new Date().getTime();
    var all_IDs = ["oat", "oap", "oah", "iat", "iap", "iah", "draftDamperPos", "draftFanPct", "waterSensor", "mainGallonsTotal", "mainGallonsToday", "mainFlowGPM", "hotGallonsTotal", "hotGallonsToday", "hotFlowGPM", ];


    myVar = setInterval(function(){
      var now = new Date().getTime();
      if(now-lastUpdated > 60000)
          all_IDs.forEach(function(item){document.getElementById(item).innerHTML="inop";});

    }, 15000); // check for scale update every 15 sec - "inop" if has had no update 

    const webSock = new WebSocket('ws://'+window.location.hostname+':80');
    webSock.onopen = function(evt){f_webSockOnOpen(evt);}
    webSock.onmessage = function(evt){f_webSockOnMessage(evt);}
    webSock.onerror  = function(evt){f_webSockOnError(evt);}

    function f_webSockOnMessage(evt){
        if(typeof evt.data === "string")j=JSON.parse(evt.data);
        lastUpdated = new Date().getTime();
        document.getElementById("msg1").innerHTML=evt.data;
        for(var key in j[0])
        document.getElementById(key).innerHTML=j[0][key];
    }
    function f_webSockOnOpen(evt){
                var msg = document.getElementById("msg");
                msg.style.wordWrap = "break-word";
                msg.style.color = "green";
                msg.innerHTML = "WebSock: CONNECTED"
    }
    function f_webSockOnError(evt){
                var msg = document.getElementById("msg");
                msg.style.wordWrap = "break-word";
                msg.style.color = "red";
                msg.innerHTML = "WebSock: ERROR";
                msg.style.visibility="visible";
    }
    function f_help(){
        var x=document.getElementsByName("msg");
        for(var i=0;i<x.length;i++)
            x[i].style.visibility=x[i].style.visibility=="hidden"?"visible":"hidden";
    }
  </script>
</html>
)=====";

extern const char SSIDPWD_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta name="file" content="set_bounds.html">
    <meta name="author" content="Joe Belson 20210822">
    <meta name="what" content="esp8266 html for WiFi connected temp/humidity bounds.">
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Set Temp / Humidity Bounds for 110VAC relays</title>
  </head>


  <body style="text-align: left;">
    <hr>
      <table class="calibrate">
        <caption style="font-weight: bolder;">Set Temp & Humidty Relays SSID : PWD</caption>
          <thead>
              <tr>
                  <th>SSID</th>
                  <th>PWD</th>
              </tr>
          </thead>
          <tbody>
              <tr>
                  <td><input type="text" id="SSID" name="cred"></td>
                  <td><input type="text" id="PWD" name="cred"></td>
                  <td><input type="submit" id="submit" onclick="f_submit()"></td>
              </tr>
          </tbody></table>


      <br>
      <label for="dhcp">DHCP:</label><input type="checkbox" id="isDHCP" onclick="f_dhcp(checked)">
      <br><br>

      <label for="ip"      name="dhcp">IP: </label>     <input type="text"  id="IP"      name="dhcp" style="width: 7rem;">
      <label for="gateway" name="dhcp">Gateway: </label><input type="text" id="GW" name="dhcp" style="width: 7rem;">
      <label for="mask"    name="dhcp">Mask: </label>   <input type="text" id="MASK"    name="dhcp" style="width: 7rem;">
      <br><p id="status"></p>
    </body>
    <script language = "javascript" type = "text/javascript">
  
        let webSock       = new WebSocket('ws://'+window.location.hostname+':80');
        webSock.onopen    = function(evt){f_webSockOnOpen(evt);}
        webSock.onmessage = function(evt){f_webSockOnMessage(evt);}
        webSock.onerror   = function(evt){f_webSockOnError(evt);}
        var str= '{"SSID":"0","PWD":"0", "IP":"0","GW":"0","MASK":"0","isDHCP":false}';
  
        function f_webSockOnMessage(evt){
          if(typeof evt.data === "string"){
              document.getElementById("status").innerHTML=evt.data;
              const j=JSON.parse(evt.data);
              const n=document.getElementsByName("dhcp");
              for(var i=0;i<n.length;i++)
                if(n[i].value="")n[i].value=j[n[i].id];
          }
        }
        function f_webSockOnOpen(evt){}
        function f_webSockOnError(evt){}
        function f_submit(){
          j=JSON.parse(str);
          for(var key in j){
            if(key=="isDHCP")j[key]=document.getElementById(key).checked;
            else j[key]=document.getElementById(key).value;
          }
          console.log(j);
          webSock.send(JSON.stringify(j));
        }
        function f_dhcp(checked){
          const n=document.getElementsByName("dhcp");
          if(checked)for(var i=0;i<n.length;i++)n[i].setAttribute("hidden", "hidden");
          else for(var i=0;i<n.length;i++)n[i].removeAttribute("hidden");
        }
     </script>
  </html>
)=====";
extern const char SETTINGS_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta name="file" content="set_bounds.html">
    <meta name="author" content="Joe Belson 20210822">
    <meta name="what" content="esp8266 html for WiFi connected temp/humidity bounds.">
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Set Temp / Humidity Bounds for 110VAC relays</title>
  </head>


  <body style="text-align: left;">
    <hr>
      <table class="calibrate">
        <caption style="font-weight: bolder;">Set Temp & Humidty Relays SSID : PWD</caption>
          <thead>
              <tr>
                  <th>SSID</th>
                  <th>PWD</th>
              </tr>
          </thead>
          <tbody>
              <tr>
                  <td><input type="text" id="SSID" name="cred"></td>
                  <td><input type="text" id="PWD" name="cred"></td>
                  <td><input type="submit" id="submit" onclick="f_submit()"></td>
              </tr>
          </tbody></table>


      <br>
      <label for="dhcp">DHCP:</label><input type="checkbox" id="isDHCP" onclick="f_dhcp(checked)">
      <br><br>

      <label for="ip"      name="dhcp">IP: </label>     <input type="tel"  id="ip"      name="dhcp" style="width: 7rem;">
      <label for="gateway" name="dhcp">Gateway: </label><input type="text" id="gateway" name="dhcp" style="width: 7rem;">
      <label for="mask"    name="dhcp">Mask: </label>   <input type="text" id="mask"    name="dhcp" style="width: 7rem;">
      <br><p id="status"></p>
    </body>
    <script language = "javascript" type = "text/javascript">
  
        let webSock       = new WebSocket('ws://'+window.location.hostname+':80');
        webSock.onopen    = function(evt){f_webSockOnOpen(evt);}
        webSock.onmessage = function(evt){f_webSockOnMessage(evt);}
        webSock.onerror   = function(evt){f_webSockOnError(evt);}
        var str= '{"SSID":"0","PWD":"0", "ip":"0","gateway":"0","mask":"0","isDHCP":false}';
  
        function f_webSockOnMessage(evt){
          if(typeof evt.data === "string"){
              document.getElementById("status").innerHTML=evt.data;
              const j=JSON.parse(evt.data);
              const n=document.getElementsByName("dhcp");
              for(var i=0;i<n.length;i++)
                if(n[i].value="")n[i].value=j[n[i].id];
          }
        }
        function f_webSockOnOpen(evt){}
        function f_webSockOnError(evt){}
        function f_submit(){
          j=JSON.parse(str);
          for(var key in j){
            if(key=="isDHCP")j[key]=document.getElementById(key).checked;
            else j[key]=document.getElementById(key).value;
          }
          console.log(j);
          webSock.send(JSON.stringify(j));
        }
        function f_dhcp(checked){
          const n=document.getElementsByName("dhcp");
          if(checked)for(var i=0;i<n.length;i++)n[i].setAttribute("hidden", "hidden");
          else for(var i=0;i<n.length;i++)n[i].removeAttribute("hidden");
        }
     </script>
  </html>
)=====";
