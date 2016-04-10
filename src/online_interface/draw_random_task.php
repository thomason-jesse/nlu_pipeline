<?php

if (!isset($_POST)) {
	die("FAILED\nno POST detected");
}
if (!isset($_POST['user_id'])) {
	die("FAILED\nno user_id posted");
}
if (!isset($_POST['task'])) {
	die("FAILED\nno task posted");
}
if (!isset($_POST['train_or_test'])) {
	die("FAILED\nno train/test set option posted");
}

$task_type = $_POST['task'];
$user_id = $_POST['user_id'];
$train_or_test = $_POST['train_or_test'];

$people_to_expressions = array('mallory'=>array('Mallory','Mallory Morgan','Dr. Morgan','The director'),
                                    'walter'=>array('Walter','Walter Ward','The supervisor'),
                                    'peggy'=>array('Peggy','Peggy Parker'),
                                    'alice'=>array('Alice','Alice Ashcraft','The secretary'),
                                    'bob'=>array('Robert','Robert Brown','Bob'),
                                    'carol'=>array('Carol','Carol Clark'),
                                    'dave'=>array('Dave','Dave Daniel'),
                                    'eve'=>array('Evelyn','Evelyn Eckhart','Eve'),
                                    'frannie'=>array('Francis','Francis Foster','Frannie'),
                                    'george'=>array('George','George Green','The intern'));
$people_to_expressions_lc = array('mallory'=>array('Mallory','Mallory Morgan','Dr. Morgan','the director'),
                                    'walter'=>array('Walter','Walter Ward','the supervisor'),
                                    'peggy'=>array('Peggy','Peggy Parker'),
                                    'alice'=>array('Alice','Alice Ashcraft','the secretary'),
                                    'bob'=>array('Robert','Robert Brown','Bob'),
                                    'carol'=>array('Carol','Carol Clark'),
                                    'dave'=>array('Dave','Dave Daniel'),
                                    'eve'=>array('Evelyn','Evelyn Eckhart','Eve'),
                                    'frannie'=>array('Francis','Francis Foster','Frannie'),
                                    'george'=>array('George','George Green','the intern'));                                    
$person_to_possessive = array('alice'=>'her','bob'=>'his','carol'=>'her','dave'=>'his','eve'=>'her','frannie'=>'her','george'=>'his','mallory'=>'her','peggy'=>'her','walter'=>'his');
$person_to_obj_pronoun = array('alice'=>'her','bob'=>'him','carol'=>'her','dave'=>'him','eve'=>'her','frannie'=>'her','george'=>'him','mallory'=>'her','peggy'=>'her','walter'=>'him');
$person_to_pronoun = array('alice'=>'she','bob'=>'he','carol'=>'she','dave'=>'he','eve'=>'she','frannie'=>'she','george'=>'he','mallory'=>'she','peggy'=>'she','walter'=>'he');
$rooms = array('l3_502', 'l3_414b', 'l3_420', 'l3_432', 'l3_404', 'l3_508', 'l3_510', 'l3_512');
$rooms_to_numbers = array('l3_502' => '3502', 'l3_414b' => '3414b', 'l3_420' => '3420', 'l3_432' => '3432', 'l3_404' => '3404', 'l3_508' => '3508', 'l3_510' => '3510', 'l3_512' => '3512');
$rooms_to_people = array('l3_502' => 'alice', 'l3_414b' => 'bob', 'l3_420' => 'carol', 'l3_432' => 'dave', 'l3_404' => 'frannie', 'l3_508' => 'mallory', 'l3_510' => 'peggy', 'l3_512' => 'walter');
$people = array('alice', 'bob', 'carol', 'dave', 'eve', 'frannie', 'george', 'mallory', 'peggy', 'walter');
$people_with_offices = array('alice', 'carol', 'dave', 'peggy', 'frannie', 'mallory', 'walter');
$foods = array('muffin', 'chips', 'coffee', 'hamburger', 'juice');
$foods_to_needs = array('muffin'=>'1','chips'=>'2','coffee'=>'3','hamburger'=>'4','juice'=>'5');

// Hold out 20% of each type of task for test
$walk_test = array('l3_404', 'l3_510');
$deliver_test = array('alice' => 'coffee', 'bob' => 'juice', 'carol' => 'muffin', 'dave' => 'muffin', 'eve' => 'coffee', 'frannie' => 'hamburger', 'george' => 'juice', 'mallory' => 'chips', 'peggy' => 'hamburger', 'walter' => 'chips');
$search_test = array('alice' => 'frannie', 'bob' => 'mallory', 'carol' => 'walter', 'dave' => 'alice', 'eve' => 'dave', 'frannie' => 'carol', 'george' => 'peggy', 'mallory' => 'alice', 'peggy' => 'carol', 'walter' => 'carol');

//draw deliver task
if (strcmp($task_type,"deliver") == 0) {
    $person = $people[rand(0,count($people)-1)];
    if ($train_or_test == 'train') {
        $food = $foods[rand(0,count($foods)-1)];        
        while ($food == $deliver_test[$food]) {
            // Retry till you get a (person, food) pair not in the test set
            $food = $foods[rand(0,count($foods)-1)];
        }
    } else {
        $food = $deliver_test[$food];
    }
    
	$target_command = "bring(".$food.",".$person.")";
	$task_description = $people_to_expressions[$person][rand(0,count($people_to_expressions[$person])-1)]." wants the item in slot ".$foods_to_needs[$food].".";
}

//draw walking task
elseif (strcmp($task_type,"at") == 0) {
    if ($train_or_test == 'train') {
        $room = $rooms[rand(0,count($rooms)-1)];  
        while (in_array($room, $walk_test)) {
            // Get a room not in the test set
            $room = $rooms[rand(0,count($rooms)-1)];  
        }
    } else {
        $room = $walk_test[rand(0,count($walk_test)-1)];
    }
	//choose a referring expression for owner of the room and design query
	if (isset($rooms_to_people[$room])) {
		$person = $rooms_to_people[$room];						
		$referring_expression = $people_to_expressions[$person][rand(0,count($people_to_expressions[$person])-1)];
		
		$target_command = "at(".$room.")";
		$task_description = $referring_expression." needs the robot. Send it to ".$person_to_possessive[$person]." office.";
	}
}

//draw search task
elseif (strcmp($task_type,"search") == 0) {
    $searchee = $people[rand(0,count($people)-1)]; // person to search for
    if ($train_or_test == 'train') {
        $owner = $people_with_offices[rand(0,count($people_with_offices)-1)]; // owner fo office to search
        while ($searchee == $owner || $owner == $search_test[$searchee]) {
            // Ensure this is not a test pair
            // Disallow the case when they are the same as it makes formulating 
            // the prompt tricky
            $searchee = $people[rand(0,count($people)-1)];
            $owner = $people_with_offices[rand(0,count($people_with_offices)-1)];
        }           
    } else {
        $owner = $search_test[$searchee];
    }
        
    // Find the room to be searched
    $room = False;
	foreach ($rooms_to_people as $key => $value) {
		if ($value == $owner) {
			$room = $key;
			break;
		}
	}
    $target_command = "search(".$searchee.','.$room.")";
    $searchee_expression = $people_to_expressions_lc[$searchee][rand(0,count($people_to_expressions_lc[$searchee])-1)];
    $owner_expression = $people_to_expressions_lc[$owner][rand(0,count($people_to_expressions_lc[$owner])-1)];
    $task_description = "Make the robot find out whether ".$searchee_expression." is in ".$owner_expression."'s office";
}

//draw query task (will be used as validation task against spammers)
elseif (strcmp($task_type,"query") == 0) {
	//choose a person
	$person = $people_with_offices[rand(0,count($people_with_offices)-1)];
	//get the person's office number
	$room = False;
	foreach ($rooms_to_people as $key => $value) {
		if ($value == $person) {
			$room = $key;
			break;
		}
	}
	$target_command = $people_to_expressions[$person][1]; #instead of query(person,n) write first and last name to ease validation
	$task_description = "Give only the first and last name of the person in office number ".$rooms_to_numbers[$room].".";
}

//unrecognized task; throw error
else {
	die("FAILED\nunknown task specified = ".$task_type);
}

//write chosen task to file
$input_file = fopen('target_commands/'.$user_id.'.txt', 'w');
fwrite($input_file, $target_command);
fclose($input_file);

//write task description to html
echo $task_description;

?>
