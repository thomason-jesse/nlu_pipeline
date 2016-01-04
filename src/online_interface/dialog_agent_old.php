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

$path_to_rlg_root = '/home/aishwarya/Documents/UT/Research/rlg';
$path_to_dialog = '/home/aishwarya/Documents/UT/Research/rlg/dialog/';
$user_id = $_POST['user_id'];

//write user input to file
$user_input = '';
if (isset($_POST['user_input']))
{
	$user_input_raw = $_POST['user_input'];
	$user_input = htmlspecialchars($user_input_raw);
	$input_file = fopen($path_to_dialog.'offline_data/inputs/'.$user_id.'_input.txt', 'w');
	fwrite($input_file, $user_input);
	fclose($input_file);
}

//if this is validation task, write to validations/ and then stop
if (strcmp($_POST['task'],'query') == 0)
{
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
	echo "Happy to help";
	die();
}

//invoke dialog agent
$system_call = 'python '.$path_to_dialog.'main.py '.$path_to_dialog.' offline '.$user_id; //.' 2> stderr.txt';
$response = exec($system_call, $output, $retval);
if ($response == FALSE)
{
	die("FAILED\nsystem call '".$system_call."' failed with '".$response."', output '".print_r($output,True)."', and retval '".$retval."'");
}

//expand permissions on all created files
$system_call = 'chmod -R 777 '.$path_to_rlg_root;
$response = exec($system_call, $output, $retval);

//read system output from file
$output_filename = $path_to_dialog.'offline_data/outputs/'.$user_id.'_output.txt';
$output_file = fopen($output_filename, 'r');
if ($output_file == FALSE)
{
	die("FAILED\nopen file '".$output_filename."' failed");
}
$output_file_contents = fread($output_file, filesize($output_filename));
$system_output = trim($output_file_contents);

//write input/output pair to html
echo $user_input;
echo "\n";
echo $system_output;

?>
