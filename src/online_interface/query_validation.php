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

$user_id = $_POST['user_id'];
if (strcmp($_POST['task'],'query') == 0) {
	if (!isset($_POST['user_input']))
	{
		die("FAILED\nquery task had no input");
	}
    $err_str = '';
	$user_input_raw = $_POST['user_input'];
	$user_input = htmlspecialchars($user_input_raw);
	$input_file = fopen('validations/'.$user_id.'.txt', 'w');
    
    $err = error_get_last();
    if ($err !== NULL) {
        $err_str = $err_str.'fopen '.$err['message'];        
    }
    
	fwrite($input_file, $user_input);
    $err = error_get_last();
    if ($err !== NULL) {
        $err_str = $err_str.'fwrite '.$err['message'];        
    }
    
	fclose($input_file);
    $err = error_get_last();
    if ($err !== NULL) {
        $err_str = $err_str.'fclose '.$err['message'];        
    }
    
	echo $user_input;
	echo "\n";
	echo "<END/>";
    //echo $err_str;
	die();
} else {
    die('query_validation invoked for a non-query task.');
}

?>
