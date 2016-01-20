<?php

if (!isset($_POST))
{
	die("FAILED\nno POST detected");
}
if (!isset($_POST['user_id']))
{
	die("FAILED\nno user_id posted");
}
if (!isset($_POST['task']))
{
	die("FAILED\nno task posted");
}
if (!isset($_POST['train_or_test']))
{
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
$person_to_possessive = array('alice'=>'her','bob'=>'his','carol'=>'her','dave'=>'his','eve'=>'her','frannie'=>'her','george'=>'his','mallory'=>'her','peggy'=>'her','walter'=>'his');
$person_to_obj_pronoun = array('alice'=>'her','bob'=>'him','carol'=>'her','dave'=>'him','eve'=>'her','frannie'=>'her','george'=>'him','mallory'=>'her','peggy'=>'her','walter'=>'him');
$person_to_pronoun = array('alice'=>'she','bob'=>'he','carol'=>'she','dave'=>'he','eve'=>'she','frannie'=>'she','george'=>'he','mallory'=>'she','peggy'=>'she','walter'=>'he');
//$rooms = array('l3_508','l3_512','l3_510','l3_404','l3_418','l3_420','l3_432','l3_502','l3_416','l3_436','l3_414b','l3_516');
//$rooms = array('l3_508','l3_512','l3_510','l3_404','l3_418','l3_420','l3_432','l3_502','l3_416','l3_436');
$rooms = array('l3_502', 'l3_402', 'l3_414b', 'l3_418', 'l3_510', 'l3_508', 'l3_512');
//$rooms_to_names = array('l3_416'=>'Meeting Room','l3_436'=>'Research Lab','l3_414b'=>'Robot Lab','l3_516'=>'Conference Room');
$rooms_to_names = array();
$rooms_to_numbers = array('l3_502' => '3502', 'l3_402' => '3402', 'l3_414b' => '3414b', 'l3_418' => '3418', 'l3_510' => '3510', 'l3_508' => '3508', 'l3_512' => '3512');
//$rooms_to_people = array('l3_416'=>'subha','l3_436'=>'katie','l3_508'=>'peter','l3_512'=>'ray','l3_510'=>'dana','l3_404'=>'kazunori','l3_418'=>'matteo','l3_420'=>'shiqi','l3_432'=>'jivko','l3_502'=>'stacy');
$rooms_to_people = array('l3_502' => 'alice', 'l3_402' => 'bob', 'l3_414b' => 'carol', 'l3_418' => 'eve', 'l3_510' => 'frannie', 'l3_508' => 'mallory', 'l3_512' => 'walter');
$people = array('alice', 'bob', 'carol', 'dave', 'eve', 'frannie', 'george', 'mallory', 'peggy', 'walter');
$people_with_offices = array('alice', 'bob', 'carol', 'eve', 'frannie', 'mallory', 'walter');
$foods = array('muffin', 'chips', 'coffee', 'hamburger', 'juice');
$foods_to_needs = array('muffin'=>'1','chips'=>'2','coffee'=>'3','hamburger'=>'4','juice'=>'5');

//define train/test splits (80% / 20%)
//$deliver_test = array('alice'=>array('muffin','hamburger','coffee'),
						//'carol'=>array('calendar','hamburger'),
						//'dave'=>array('calendar','trashcan'),
						//'george'=>array('calendar'),
						//'mallory'=>array('trashcan'),
						//'bob'=>array('phone'));
//$walk_test = array('l3_512','l3_416');
$deliver_test = array();
$walk_test = array();

//draw deliver task
if (strcmp($task_type,"deliver") == 0)
{	
	//choose a person and a food item until we get something in the right set
	$valid_task = False;
	while ($valid_task == False)
	{
		$person = $people[rand(0,count($people)-1)];
		$food = $foods[rand(0,count($foods)-1)];
		
		if (strcmp($train_or_test,'test') == 0 and array_key_exists($person,$deliver_test) and in_array($food,$deliver_test[$person]))
		{
			$valid_task = True;
		}
		elseif (strcmp($train_or_test,'train') == 0 and !array_key_exists($person,$deliver_test) and !in_array($food,$deliver_test[$person]))
		{
			$valid_task = True;
		}
	}
	
	$target_command = "served(".$person.",".$food.",n)";
	$task_description = $people_to_expressions[$person][rand(0,count($people_to_expressions[$person])-1)]." wants the item in slot ".$foods_to_needs[$food].".";
}

//draw walking task
elseif (strcmp($task_type,"at") == 0)
{
	
	//choose a room until we get something in the right set
	$valid_task = False;
	while ($valid_task == False)
	{
		$room = $rooms[rand(0,count($rooms)-1)];
		
		if ((strcmp($train_or_test,'test') == 0 and in_array($room,$walk_test)) or (strcmp($train_or_test,'train') == 0 and !in_array($room,$walk_test)))
		{
			$valid_task = True;
		}
	}
	
	//choose a referring expression for owner of the room and design query
	if (isset($rooms_to_people[$room]))
	{
		$person = $rooms_to_people[$room];						
		$referring_expression = $people_to_expressions[$person][rand(0,count($people_to_expressions[$person])-1)];
		
		$target_command = "at(".$room.",n)";
		$task_description = $referring_expression." needs the robot. Send it to ".$person_to_possessive[$person]." office.";
	}
	//choose a referring expression for the room and design query
	else
	{
		$target_command = "at(".$room.",n)";
		$task_description = "Send the robot to the ".$rooms_to_names[$room].".";
	}
}

//draw query task (will be used as validation task against spammers)
elseif (strcmp($task_type,"query") == 0)
{
	//choose a person
	$person = $people_with_offices[rand(0,count($people_with_offices)-1)];
	
	//get the person's office number
	$room = False;
	foreach ($rooms_to_people as $key => $value)
	{
		if ($value == $person)
		{
			$room = $key;
			break;
		}
	}
	
	$target_command = $people_to_expressions[$person][1]; #instead of query(person,n) write first and last name to ease validation
	$task_description = "Give only the first and last name of the person in office number ".$rooms_to_numbers[$room].".";
}

//unrecognized task; throw error
else
{
	die("FAILED\nunknown task specified = ".$task_type);
}

//write chosen task to file
$input_file = fopen('target_commands/'.$user_id.'.txt', 'w');
fwrite($input_file, $target_command);
fclose($input_file);

//write task description to html
echo $task_description;

?>
