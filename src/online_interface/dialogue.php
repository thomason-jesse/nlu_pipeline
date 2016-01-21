<HTML>

<HEAD>
<TITLE>Robot Language Grounding</TITLE>
<META http-equiv="Content-Type" content="text/html; charset=utf-8" />

<script type="text/javascript" src="http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js"></script>
<script type="text/javascript" src="http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js"></script>

<SCRIPT LANGUAGE="JavaScript" >

	var train_or_test = "train";
	var user_id_base = null; //supplied from MTurk
	var user_id = null; //changes based on task
	var current_task = null;
	
	//what little style exists
	var user_cell_color = 'AliceBlue';
	var system_cell_color = 'GhostWhite';

    // Set up rosbridge connecton
    var ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
    });

    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        //alert('Connected to websocket server.');
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        alert('Error connecting to websocket server: ' + error);
    });

    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        alert.log('Connection to websocket server closed.');
    });

    // These will point to different topics as the dialogue progresses
    var publisher = null;
    var subscriber = null;
    
    // Global variables to talk to the publisher
    var publish_msg = null;
    var publish_interval = 100;
    var publishing = false;
    var seq_no = 0;
    
    // Global variables for subscriber
    var subscriber_prev_msg = null;

    function decideTask() {
        var tasks = ['walk', 'deliver', 'search'];
        var task = tasks[Math.floor(Math.random()*tasks.length)];
        if (task == 'walk') {
            drawRandomWalkTask();
        } else if (task == 'deliver') {
            drawRandomDeliverTask();
        } else {
            drawRandomSearchTask();
        }
        
    }

	//draw a random walk task and return the description to javascript; write the task goal to file for later comparison against command generated
	function drawRandomWalkTask()
	{
		//get user id
		user_id_base = <?php echo "'".uniqid()."'";?>;
		user_id = user_id_base.concat('_main');
		current_task = "main";
	
		//hide ask id div and open task show div
		document.getElementById('ask_for_ID').style.display = 'none';
		document.getElementById('inst').style.display = 'block';
		document.getElementById('introduce_task').style.display = 'block';
		document.getElementById('task_map').style.display = 'block';
		document.getElementById('dialog_start_block').style.display = 'block';
	
		getRequest(
			  'draw_random_task.php', // URL for the PHP file
			  'task=at&user_id='.concat(user_id).concat('&train_or_test=').concat(train_or_test), // parameters for PHP
			   invokeRandomTaskOutput,  // handle successful request
			   invokeRandomTaskError    // handle error
		  );
	}
	
	//draw a random deliver task and return the description to javascript; write the task goal to file for later comparison against command generated
	function drawRandomDeliverTask()
	{
		//update user id for new task
		//get user id
		user_id_base = <?php echo "'".uniqid()."'";?>;
		user_id = user_id_base.concat('_main');
		current_task = "main";
	
		//hide ask id div and open task show div
		document.getElementById('ask_for_ID').style.display = 'none';
		document.getElementById('inst').style.display = 'block';
		document.getElementById('introduce_task').style.display = 'block';
		document.getElementById('task_map').style.display = 'block';
		document.getElementById('dialog_start_block').style.display = 'block';
	
		getRequest(
			  'draw_random_task.php', // URL for the PHP file
			  'task=deliver&user_id='.concat(user_id).concat('&train_or_test=').concat(train_or_test), // parameters for PHP
			   invokeRandomTaskOutput,  // handle successful request
			   invokeRandomTaskError    // handle error
		  );
	}
    
    //draw a random search task and return the description to javascript; write the task goal to file for later comparison against command generated
	function drawRandomSearchTask()
	{
        //get user id
		user_id_base = <?php echo "'".uniqid()."'";?>;
		user_id = user_id_base.concat('_main');
		current_task = "main";
	
		//hide ask id div and open task show div
		document.getElementById('ask_for_ID').style.display = 'none';
		document.getElementById('inst').style.display = 'block';
		document.getElementById('introduce_task').style.display = 'block';
		document.getElementById('task_map').style.display = 'block';
		document.getElementById('dialog_start_block').style.display = 'block';
	
        getRequest(
			  'draw_random_task.php', // URL for the PHP file
			  'task=search&user_id='.concat(user_id).concat('&train_or_test=').concat(train_or_test), // parameters for PHP
			   invokeRandomTaskOutput,  // handle successful request
			   invokeRandomTaskError    // handle error
		  );
	}
	
	//draw a random query task and return the description to javascript; write the task goal to file for later comparison against command generated; this is the validation task to detect spammers
	function drawRandomQueryTask()
	{
		//update user id for new task
		user_id = user_id_base.concat('_query');
		current_task = "query";
	
		getRequest(
			  'draw_random_task.php', // URL for the PHP file
			  'task=query&user_id='.concat(user_id).concat('&train_or_test=').concat(train_or_test), // parameters for PHP
			   invokeRandomTaskOutput,  // handle successful request
			   invokeRandomTaskError    // handle error
		  );
	}
	
	// handles the response, adds the html
	function invokeRandomTaskOutput(response_text) {
	
		//alert('DEBUG: output from php:\n'.concat(response_text));
	
		var task_description_text = document.getElementById('task_description_text');
		task_description_text.innerHTML = response_text;

	}
	
	// handles drawing an error message
	function invokeRandomTaskError () {
		alert('DEBUG: there was an error calling draw_random_task.php');
	}
    
    // Publish the current message 
    function publish() {
       if (publisher !== null && publish_msg !== null && publishing) {
            var msg = new ROSLIB.Message({
                data : publish_msg
            });
            //alert('Publishing ' + publish_msg);
            publisher.publish(msg);
       }
    }
    
    //run to start a dialog with the system for a new user
	function startDialog()
	{
		//alert('DEBUG: startDialog() called');
		
		//hide start div and open dialog div
		document.getElementById('dialog_start_block').style.display = 'none';
		document.getElementById('dialog_history_block').style.display = 'block';
		
		//gray out text area to prevent overloading system while it's starting up
		document.getElementsByName("user_input_box")[0].disabled = true;
		
		//add robot initial [thinking...] cell
		var table = document.getElementsByName('history')[0];
		var system_row = table.insertRow(table.rows.length-1);
		var system_response_cell = system_row.insertCell(0);
		system_response_cell.innerHTML = "<i>typing...</i>";
		system_response_cell.style.backgroundColor = system_cell_color;
		var system_name_cell = system_row.insertCell(0);
		system_name_cell.innerHTML = "ROBOT";
		system_name_cell.style.backgroundColor = system_cell_color;
        
        //alert('ros = ' + ros + ' user_id = ' + user_id);

        // Set up a subscriber to listen to the dialog agent
        subscriber = new ROSLIB.Topic({
                            ros : ros,
                            name : 'python_pub_' + user_id,
                            messageType : 'std_msgs/String'
                        });

        subscriber.subscribe(dialogResponseReceiver);
        //alert('Subscriber created');
        
        // Publish id continuously 
        publisher = new ROSLIB.Topic({
                        ros : ros,
                        name : '/new_user_topic',
                        messageType : 'std_msgs/String'
                    });
        publish_msg = user_id
        publishing = true;
        setInterval(publish, 100);        
        //alert('Publishing started');
        
		return false;
	}
    
    function generalError(msg) {
        alert('Error' + msg);
    }
    
    function onCreateSocket(response) {
        var response_obj = eval(response)
        if (response_obj[0] != null) {
            alert('Error: ' + response_obj[0]);
            return;
        }
        //alert(response_obj[1]);
        socket = response_obj[1];
    }
	
	function startThirdDialog()
	{
		//alert('DEBUG: startThirdDialog() called');
		
		//change map
		document.getElementById('task_map').innerHTML = "<b>DIRECTORY</b><p><img src=\"directory.png\" alt=\"Rooms, offices and labs\" style=\"width:60%\"></p>";
		
		//hide start div and clear dialog table of past conversation
		document.getElementById('third_dialog_start_block').style.display = 'none';
		table = document.getElementsByName('history')[0];
		while (table.rows.length > 1)
		{
			table.deleteRow(0);
		}
		
		//add robot cell
		var table = document.getElementsByName('history')[0];
		var system_row = table.insertRow(table.rows.length-1);
		var system_response_cell = system_row.insertCell(0);
		system_response_cell.innerHTML = "Please write the first and last name only.";
		system_response_cell.style.backgroundColor = system_cell_color;
		var system_name_cell = system_row.insertCell(0);
		system_name_cell.innerHTML = "ROBOT";
		system_name_cell.style.backgroundColor = system_cell_color;
		
		//ungray text area to allow user to respond
		document.getElementsByName("user_input_box")[0].disabled = false;
	
		//draw a new task to display; updates task variables and displays new description
		drawRandomQueryTask();
		
		//update instructions text to be a little clearer
		document.getElementById('inst').innerHTML = "You can use the directory to look up room numbers and find the full name requested.";
		
		return false;
	}

	//run when the user submits a new response to the system
	function getDialogResponse(form)
	{
        //alert('In getDialogResponse, current_task =  ' + current_task);
		//get user input and clear text box
		var user_input_raw = form.user_input_box.value;
		form.user_input_box.value = '';
		
		//do initial sanitization of user text and ignore command if it's empty/bogus
		var temp_div = document.createElement("div");
		temp_div.innerHTML = user_input_raw;
		var user_input = temp_div.textContent || temp_div.innerText || "";
		if (user_input.length == 0) return;
		
		//if user input is to be submitted, gray out text area to prevent overloading system
		document.getElementsByName("user_input_box")[0].disabled = true;
		
		//add user text
		var table = document.getElementsByName('history')[0];
		var user_row = table.insertRow(table.rows.length-1);
		var user_response_cell = user_row.insertCell(0);
		user_response_cell.innerHTML = user_input;
		user_response_cell.style.backgroundColor = user_cell_color;
		var user_name_cell = user_row.insertCell(0);
		user_name_cell.innerHTML = "YOU";
		user_name_cell.style.backgroundColor = user_cell_color;
		
		//add robot initial [thinking...] cell
		var system_row = table.insertRow(table.rows.length-1);
		var system_response_cell = system_row.insertCell(0);
		system_response_cell.innerHTML = "<i>typing...</i>";
		system_response_cell.style.backgroundColor = system_cell_color;
		var system_name_cell = system_row.insertCell(0);
		system_name_cell.innerHTML = "ROBOT";
		system_name_cell.style.backgroundColor = system_cell_color;

        if (current_task == 'query') {
            //submit php request
            getRequest(
              'query_validation.php', // URL for the PHP file
              'task='.concat(current_task).concat('&user_input=').concat(user_input).concat('&user_id=').concat(user_id), // parameters for PHP
               invokeDialogAgentOutput,  // handle successful request
               invokeDialogAgentError    // handle error
            );
        } else {
            // Set up a subscriber to listen to the dialog agent
            subscriber = new ROSLIB.Topic({
                                ros : ros,
                                name : 'python_pub_' + user_id,
                                messageType : 'std_msgs/String'
                            }); // This is probably unnecessary
            subscriber.subscribe(dialogResponseReceiver);
            
            // Publish user response
            publisher = new ROSLIB.Topic({
                            ros : ros,
                            name : 'js_pub_' + user_id,
                            messageType : 'std_msgs/String'
                        });
            publish_msg = seq_no + '|' + user_input
            seq_no = seq_no + 1;
            publishing = true;     
        }
	}

	// handles drawing an error message
	function invokeDialogAgentError () {
		alert('DEBUG: there was an error calling dialog_agent.php');
	}

    function dialogResponseReceiver(msg) {
        text = msg.data;
        if (text.indexOf('|') <= -1) {
            alert('Malformed string');
            return;
        }
        
        if (publish_msg === user_id) {
            // This is for the first response of a task. Stop publishing id
            publishing = false;
        }
        
        if (text !== subscriber_prev_msg) {
            // Since js does not have a lock, this could be a problem. 
            subscriber.unsubscribe();
            subscriber_prev_msg = text;
            //alert('Received ' + text);
            var parts = text.split('|');
            //alert('parts = ' + parts);
            var response_text = parts[1] + '\n' + parts[2];
            //alert('response_text = ' + response_text);
            invokeDialogAgentOutput(response_text);
        }
    }

	// handles the response, adds the html
	function invokeDialogAgentOutput(response_text) {
        //alert('In invokeDialogAgentOutput: Response = ' + response_text);
		//alert('DEBUG: output from php:\n'.concat(response_text));
	
		//split input and output from php response
		var input_output_pair = response_text.split("\n");
		user_input = input_output_pair[0]; //sanitized
		if (user_input == "FAILED")
		{
			alert('ERROR: something failed when invoking the dialog agent');
			alert('DEBUG: '.concat(response_text));
			return;
		}
		system_output = input_output_pair.slice(1,input_output_pair.length); //system response(s)
		
		//append system output to page conversation log
		var table = document.getElementsByName('history')[0];
		//for first line of system output, overwrite [thinking...] line already created with text
		var system_row = table.rows[table.rows.length-2];
		var system_response_cell = system_row.cells[1];
		system_response_cell.innerHTML = system_output[0];
		//add remaining lines of system output, if any
		for (i = 1; i < system_output.length; i++)
		{
			var system_row = table.insertRow(table.rows.length-1);
			var system_response_cell = system_row.insertCell(0);
			system_response_cell.innerHTML = system_output[i];
			system_response_cell.style.backgroundColor = system_cell_color;
			var system_name_cell = system_row.insertCell(0);
			system_name_cell.innerHTML = "ROBOT";
			system_name_cell.style.backgroundColor = system_cell_color;
		}
		
		//if the dialog has concluded
		if (system_output[system_output.length-1] == "<END/>")
		{
			//hide the table's final row (the one with the form for input)
			
			//make visible the DIV to start the next task
			//if (current_task == "walk")
			//{
				//document.getElementById('second_dialog_start_block').style.display = 'block';
			//}
			
			////make visible the DIV to start the final task
			//else if (current_task == "deliver")
			//{
				//document.getElementById('third_dialog_start_block').style.display = 'block';
			//}
			
			//end session and give user code for MTurk
			if (current_task == "query")
			{
				document.getElementById('end_session_block').style.display = 'block';
			}
            else
            {
				document.getElementById('third_dialog_start_block').style.display = 'block';
			}
		}
		else
		{
			//re-activate user's textarea so a response can be typed
			document.getElementsByName("user_input_box")[0].disabled = false;
		}
	}
	
	function endSession()
	{
		document.getElementById('wrap').style.display = 'none';
		document.getElementById('inst').innerHTML = 'Please fill out this brief survey about your experience with the system.';
		document.getElementById('survey_block').style.display = 'block';
        
        //callPhp('close_socket', [socket], NULL, generalError)
	}
	
	//invoke php to record likert and produce code for MTurk
	function submitSurvey(form)
	{
		user_id = user_id_base;
		var comment_raw = document.getElementById('user_survey_comment_box').value;
		var temp_div = document.createElement("div");
		temp_div.innerHTML = comment_raw;
		var comment_text = temp_div.textContent || temp_div.innerText || "";
	
		var easy_selection = -1;
		for (var i = 0; i < form.easy.length; i++) {
			if (form.easy[i].checked)
			{
				easy_selection = i;
				break;
			}
		}
		var understand_selection = -1;
		for (var i = 0; i < form.understand.length; i++) {
			if (form.understand[i].checked)
			{
				understand_selection = i;
				break;
			}
		}
		var frustrate_selection = -1;
		for (var i = 0; i < form.frustrate.length; i++) {
			if (form.frustrate[i].checked)
			{
				frustrate_selection = i;
				break;
			}
		}
		if (easy_selection == -1 || understand_selection == -1 || frustrate_selection == -1)
		{
			alert('Please answer all three survey questions.');
			return false;
		}
		
		//hide instructions
		document.getElementById('inst').style.display = 'none';
		
		//submit php request
		getRequest(
		  'submit_survey.php', // URL for the PHP file
		  'user_id='.concat(user_id).concat('&easy=').concat(easy_selection).concat('&understand=').concat(understand_selection).concat('&frustrate=').concat(frustrate_selection).concat('&comment=').concat(comment_text), // parameters for PHP
		   submitSurveyOutput,  // handle successful request
		   submitSurveyError    // handle error
		);
	}
	
	// handles the response, adds the html
	function submitSurveyOutput(response_text)
	{
		document.getElementById('survey_block').innerHTML = response_text;
	}
	
	// handles drawing an error message
	function submitSurveyError () {
		alert('DEBUG: there was an error calling submit_survey.php');
	}
    
    function callPhp(function_name, args, on_success, on_error) {
        if (typeof on_success != 'function') on_success = function () {};
		if (typeof on_error!= 'function') on_error = function () {};
        var params = 'functionname='.concat(function_name)
        if (args.length > 0) {
            params = params.concat('&arg1=').concat(args[0])
        }
        if (args.length > 1) {
            params = params.concat('&arg2=').concat(args[1])
        }

        getRequest(
			  'dialog_agent.php', // URL for the PHP file
               params , // parameters for PHP
			   on_success,  // handle successful request
			   on_error    // handle error
		  );
        //$.ajax({
            //type: "POST",
            //url: 'dialog_agent.php',
            //dataType: 'json',
            //data: {functionname: function_name, arguments: args},

            //success: function (obj, textstatus) {
                          //if( !('error' in obj) ) {
                              //on_success(obj.result);
                          //}
                          //else {
                              //on_error(obj.error);
                          //}
                    //}
        //});
        
    }
	
	// helper function for cross-browser request object
	function getRequest(url, params, success, error)
	{
		//alert('DEBUG: getRequest called with url '.concat(url).concat(' and params ').concat(params));
		
		//encode params
		var params_enc = encodeURI(params)
	
		var req = false;
		try{
			// most browsers
			req = new XMLHttpRequest();
		} catch (e){
			// IE
			try{
				req = new ActiveXObject("Msxml2.XMLHTTP");
			} catch (e) {
				// try an older version
				try{
					req = new ActiveXObject("Microsoft.XMLHTTP");
				} catch (e){
					return false;
				}
			}
		}
		if (!req) return false;
		if (typeof success != 'function') success = function () {};
		if (typeof error!= 'function') error = function () {};
		req.onreadystatechange = function(){
			if(req.readyState == 4){
				return req.status === 200 ? 
					success(req.responseText) : error(req.status)
				;
			}
		}
		req.open("POST", url, true);
		req.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
		req.send(params_enc);
		return req;
	}
</SCRIPT>
	
<STYLE>
#inst {
	width:100%
	float:top;
}
#wrap {
	width:100%;
	margin:0 auto;
}
#left {
	float:left;
	width:50%;
}
#right {
float:right;
width:50%
}
</STYLE>

</HEAD>

<BODY>

<p>
<DIV ID="warning" style="color:red" >
	Do not navigate away from or refresh the page until you have completed all tasks and the exit survey to receive your code. Leaving or refreshing the page <b>will</b> prevent you from completing this HIT.
</DIV>
</p>

<p>
<DIV ID="inst" style="display:none">
	Instruct the robot so that the task specified is completed. The robot can make plans about how to accomplish the goals you give it, so you just need to tell it at a high level what to do.
</DIV>
</p>

<p>
<DIV ID="wrap">
<DIV ID="left">
	<DIV ID="ask_for_ID">
		<p>You will have a text conversation with a robot that knows how to perform the following tasks -  
        <ul>
          <li>Walk to a location. </li>
          <li>Bring an item for someone. </li>
          <li>Search a room for someone. </li>
        </ul>
        </p>
        <p> You need to do the following - 
        <ol>
          <li>Instruct the robot to accomplish a task, clarifying any questions it has. </li>
          <li>Name a person on the floor to verify that you are a human. </li>
          <li>Complete a small survery about your experiences. </li>
          <li>Receive your code for Mechanical Turk. </li>
        </ol>
        </p>
        
        <p> You need to complete all the above steps to complete the HIT. Navigating away from the page or refreshing it during any of the above steps <b>will</b> prevent you from completing the HIT. If the conversation extends too long, you may end it by replying 'stop' but ending the conversation before ten responses from your side without successfully communicating the task <b>will</b> invalidate your HIT.  </p>
        
		<p><FORM NAME="ask_for_ID_form" ACTION="" METHOD="GET">
			Click the button below to begin.<br/>
			<INPUT TYPE="button" NAME="user_id_button" Value="Begin" onClick="decideTask()">
		</FORM></p>
	</DIV>

	<DIV ID="introduce_task" style="display:none">
		<b>TASK TO COMPLETE</b><p ID="task_description_text"></p>
	</DIV>
	
	<DIV ID="dialog_start_block" style="display:none">
		<FORM NAME="user_start_dialog_form" ACTION="" METHOD="GET">
			<INPUT TYPE="button" NAME="user_start_dialog_button" Value="Start Task" onClick="startDialog()">
		</FORM>
	</DIV>

	<DIV ID="dialog_history_block" style="display:none">
		<TABLE NAME="history" style="width:100%">
		<FORM NAME="user_input_form" ACTION="" METHOD="GET">
		<TR ID="user_input_table_row">
			<TD td style="width:15%">YOU</TD>
			<TD td style="width:85%">
				<INPUT TYPE="text" NAME="user_input_box" VALUE="" style="width:100%" onkeydown="if (event.keyCode == 13) {document.getElementsByName('user_input_button')[0].click();event.returnValue=false;event.cancel=true;}">
				<INPUT TYPE="button" NAME="user_input_button" Value="submit" onClick="getDialogResponse(this.form)" style="display:none">
			</TD>
		</TR>
		</FORM>
		</TABLE>
	</DIV>
	
	<DIV ID="third_dialog_start_block" style="display:none">
		<FORM NAME="user_start_third_dialog_form" ACTION="" METHOD="GET">
			<INPUT TYPE="button" NAME="user_start_third_dialog_button" Value="Validate" onClick="startThirdDialog()">
		</FORM>
	</DIV>
	
	<DIV ID="end_session_block" style="display:none">
		<FORM NAME="user_end_session_form" ACTION="" METHOD="GET">
			<INPUT TYPE="button" NAME="user_end_session_button" Value="Fill Survey" onClick="endSession()">
		</FORM>
	</DIV>

</DIV>
<DIV ID="right">

	<DIV ID="task_map" style="display:none">
		<b>People and items the robot knows - </b>
			<p><img src="bring_task_data.png" alt="Rooms, labs and offices" style="width:100%"></p>
	</DIV>
	
</DIV>
</DIV>

<DIV ID="survey_block" style="display:none">
	<FORM NAME="survey_form" ACTION="" METHOD="GET">
		<p><b>1)</b> The tasks were easy to understand.
		<TABLE style="width:50%">
			<tr align="center" bgcolor='GhostWhite'><td style="width:20%">Strongly Disagree</td><td style="width:20%">Somewhat Disagree</td><td style="width:20%">Neutral</td><td style="width:20%">Somewhat Agree</td><td style="width:20%">Strongly Agree</td></tr>
			<tr align="center" bgcolor='AliceBlue'><td style="width:20%"><INPUT TYPE="radio" name="easy" value="0"></td><td style="width:20%"><INPUT TYPE="radio" name="easy" value="1"></td><td style="width:20%"><INPUT TYPE="radio" name="easy" value="2"></td><td style="width:20%"><INPUT TYPE="radio" name="easy" value="3"></td><td style="width:20%"><INPUT TYPE="radio" name="easy" value="4"></td></tr>
		</TABLE></p>
		<p><b>2)</b> The robot understood me.
		<TABLE style="width:50%">
			<tr align="center" bgcolor='GhostWhite'><td style="width:20%">Strongly Disagree</td><td style="width:20%">Somewhat Disagree</td><td style="width:20%">Neutral</td><td style="width:20%">Somewhat Agree</td><td style="width:20%">Strongly Agree</td></tr>
			<tr align="center" bgcolor='AliceBlue'><td style="width:20%"><INPUT TYPE="radio" name="understand" value="0"></td><td style="width:20%"><INPUT TYPE="radio" name="understand" value="1"></td><td style="width:20%"><INPUT TYPE="radio" name="understand" value="2"></td><td style="width:20%"><INPUT TYPE="radio" name="understand" value="3"></td><td style="width:20%"><INPUT TYPE="radio" name="understand" value="4"></td></tr>
		</TABLE></p>
		<p><b>3)</b> The robot frustrated me.
		<TABLE style="width:50%">
			<tr align="center" bgcolor='GhostWhite'><td style="width:20%">Strongly Disagree</td><td style="width:20%">Somewhat Disagree</td><td style="width:20%">Neutral</td><td style="width:20%">Somewhat Agree</td><td style="width:20%">Strongly Agree</td></tr>
			<tr align="center" bgcolor='AliceBlue'><td style="width:20%"><INPUT TYPE="radio" name="frustrate" value="0"></td><td style="width:20%"><INPUT TYPE="radio" name="frustrate" value="1"></td><td style="width:20%"><INPUT TYPE="radio" name="frustrate" value="2"></td><td style="width:20%"><INPUT TYPE="radio" name="frustrate" value="3"></td><td style="width:20%"><INPUT TYPE="radio" name="frustrate" value="4"></td></tr>
		</TABLE></p>
		<p><b>4)</b>Feel free to leave comments on your experience (optional):<br/>
			<textarea id="user_survey_comment_box" name="comment" form="survey_form" style="width:50%" rows="4"></textarea></p>
		<INPUT TYPE="button" NAME="user_submit_survey_button" Value="Finish and get code" onClick="submitSurvey(this.form)">
	</FORM>
</DIV>

</p>

</BODY>

</HTML>
