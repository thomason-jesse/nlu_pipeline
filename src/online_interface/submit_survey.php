<?php

if (!isset($_POST))
{
	die("FAILED\nno POST detected");
}
if (!isset($_POST['user_id']))
{
	die("FAILED\nno user_id posted");
}
if (!isset($_POST['easy']) || $_POST['easy'] == -1)
{
	die("FAILED\nno easy survey response posted");
}
if (!isset($_POST['understand']) || $_POST['understand'] == -1)
{
	die("FAILED\nno understand survey response posted");
}
if (!isset($_POST['delays']) || $_POST['delays'] == -1)
{
	die("FAILED\nno delays survey response posted");
}
if (!isset($_POST['sensible']) || $_POST['sensible'] == -1)
{
	die("FAILED\nno sensible survey response posted");
}
if (!isset($_POST['conv_long']) || $_POST['conv_long'] == -1)
{
	die("FAILED\nno conv_long survey response posted");
}
if (!isset($_POST['comment']))
{
	die("FAILED\nno comment survey response posted");
}

//$path_to_log = '../../Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/log/';
//$path_to_log_special = '../../Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/log_special/';
//$path_to_executed_action = '../../Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/executed_action/';
$path_to_log = 'log/';
$path_to_log_special = 'log_special/';
$path_to_executed_action = 'executed_actions/'; 

$task_type = $_POST['task'];
$user_id = $_POST['user_id'];
$easy = $_POST['easy'];
$understand = $_POST['understand'];
$delays = $_POST['delays'];
$sensible = $_POST['sensible'];
$conv_long = $_POST['conv_long'];
$comment = $_POST['comment'];

// Write user survey responses to file
$input_file = fopen('surveys/'.$user_id.'.txt', 'w');
$survey_output = $easy.",".$understand.",".$delays.",".$sensible.",".$conv_long."\n".$comment;
fwrite($input_file, $survey_output);
fclose($input_file);

// Validate user's responses
$task_success = false;
$output = '';

// Get target command
$input_filename = 'target_commands/'.$user_id.'_main.txt';
$input_file = fopen($input_filename, 'r') or die('Error: Target command log not found');
$target_command = strtolower(trim(fread($input_file, filesize($input_filename))));
fclose($input_file);

// Get executed action
$input_filename = $path_to_executed_action.$user_id.'_main.txt';
clearstatcache();
if (file_exists($input_filename)) {
    $input_file = fopen($input_filename, 'r');
    $executed_action = strtolower(trim(fread($input_file, filesize($input_filename))));
    fclose($input_file);
    
    if ($executed_action == $target_command) {
        $task_success = true;
    }
}

// If the executed action did not match the target, check if the dialog  
// terminated with 'stop' and if so, how long it was
$input_filename = $path_to_log.$user_id.'_main.txt';
$num_user_responses = 0;
$stop_used = false;

if (file_exists($input_filename)) {
    $input_file = fopen($input_filename, 'r');
    //$output = $output.'File exists <br/>';
    if ($input_file) {
        //$output = $output.'Opened file';
        while (($line = fgets($input_file, 4096)) !== false) {
            //$output = $output.$line.'<br/>';
            if (substr($line, 0, 4) == 'USER') {
                if (strtolower(trim(substr($line, 6))) == 'stop') {
                    $stop_used = true;
                }
                $num_user_responses++;
            }
        }
        fclose($input_file);
    }
}
if (!$stop_used || $num_user_responses >= 10) {
    $task_success = true;
}

// Verification of correct query entry 
$input_filename = 'validations/'.$user_id.'_query.txt';
$input_file = fopen($input_filename, 'r');
$validation_response = strtolower(trim(fread($input_file, filesize($input_filename))));
fclose($input_file);
$input_filename = 'target_commands/'.$user_id.'_query.txt';
$input_file = fopen($input_filename, 'r');
$target_validation = strtolower(trim(fread($input_file, filesize($input_filename))));
fclose($input_file);
if (strcmp($validation_response,$target_validation) != 0) {
	$task_success = false;
}

// Create Mturk code
$num = rand(49, 61);
if ($task_success) {
    $ord = 2 * $num - 1;        
} else {
    $ord = 2 * $num;    
}
$mturk_code = $user_id."_".substr(sha1("rlg_salted_hash".$user_id),0,13).chr($ord);

// Write Mturk code to file
$input_file = fopen('codes/'.$user_id.'.txt', 'w');
$file_output = $user_id.','.$mturk_code.'\n';
fwrite($input_file, $file_output);
fclose($input_file);

// Write MTurk code to output
$output = $output."<p>task_success = ".(int)$task_success;
$output = $output."</p>"."<p>Thank you for your participation!</p><p>Copy the code below, return to Mechanical Turk, and enter it to receive payment:<br/>".$mturk_code."</p>";
echo $output
?>
