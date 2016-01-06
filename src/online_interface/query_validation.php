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

if (strcmp($_POST['task'],'query') == 0) {
	if (!isset($_POST['user_input']))
	{
		die("FAILED\nquery task had no input");
	}
	$user_input_raw = $_POST['user_input'];
	$user_input = htmlspecialchars($user_input_raw);
	$input_file = fopen('validations/'.$user_id.'.txt', 'w');
	fwrite($input_file, $user_input);
	fclose($input_file);
	echo $user_input;
	echo "\n";
	echo "<END/>";
	die();
} else {
    die('query_validation invoked for a non-query task.');
}

?>
