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
if (!isset($_POST['frustrate']) || $_POST['frustrate'] == -1)
{
	die("FAILED\nno frustrate survey response posted");
}
if (!isset($_POST['comment']))
{
	die("FAILED\nno comment survey response posted");
}

$path_to_dialog = '../../users/jesse/RLG/dialog/';

$task_type = $_POST['task'];
$user_id = $_POST['user_id'];
$easy = $_POST['easy'];
$understand = $_POST['understand'];
$frustrate = $_POST['frustrate'];
$comment = $_POST['comment'];

//write user survey responses to file
$input_file = fopen('surveys/'.$user_id.'.txt', 'w');
$survey_output = $easy.",".$understand.",".$frustrate."\n".$comment;
fwrite($input_file, $survey_output);
fclose($input_file);

//validate user's responses
$walk_valid_moved = false;
$deliver_valid_moved = false;

//walk validation
$input_filename = $path_to_dialog."offline_data/commands/".$user_id."_walk_command.txt";
$input_file = fopen($input_filename, 'r');
$command = strtolower(trim(fread($input_file, filesize($input_filename))));
fclose($input_file);
$input_filename = 'target_commands/'.$user_id.'_walk.txt';
$input_file = fopen($input_filename, 'r');
$target_validation = strtolower(trim(fread($input_file, filesize($input_filename))));
fclose($input_file);
if (strcmp($command,$target_validation) != 0)
{
	//rename user's alog files so they won't be used
	$walk_valid_moved = true;
	$path_alog_prefix = $path_to_dialog."offline_data/alignments/".$user_id;
	exec("mv ".$path_alog_prefix."_walk_alog.txt ".$path_alog_prefix."_walk_alog.wrong.txt");
	//echo "walk error<br/>".$command."<br/>".$target_validation."<br/>"; #DEBUG
}

//deliver validation
$input_filename = $path_to_dialog."offline_data/commands/".$user_id."_deliver_command.txt";
$input_file = fopen($input_filename, 'r');
$command = strtolower(trim(fread($input_file, filesize($input_filename))));
fclose($input_file);
$input_filename = 'target_commands/'.$user_id.'_deliver.txt';
$input_file = fopen($input_filename, 'r');
$target_validation = strtolower(trim(fread($input_file, filesize($input_filename))));
fclose($input_file);
if (strcmp($command,$target_validation) != 0)
{
	//rename user's alog files so they won't be used
	$deliver_valid_moved = true;
	$path_alog_prefix = $path_to_dialog."offline_data/alignments/".$user_id;
	exec("mv ".$path_alog_prefix."_deliver_alog.txt ".$path_alog_prefix."_deliver_alog.wrong.txt");
	//echo "deliver error<br/>".$command."<br/>".$target_validation."<br/>"; #DEBUG
}

//query validation (can invalidate both previous dialogs)
$input_filename = 'validations/'.$user_id.'_query.txt';
$input_file = fopen($input_filename, 'r');
$validation_response = strtolower(trim(fread($input_file, filesize($input_filename))));
fclose($input_file);
$input_filename = 'target_commands/'.$user_id.'_query.txt';
$input_file = fopen($input_filename, 'r');
$target_validation = strtolower(trim(fread($input_file, filesize($input_filename))));
fclose($input_file);
if (strcmp($validation_response,$target_validation) != 0)
{
	//rename user's alog files so they won't be used
	$path_alog_prefix = $path_to_dialog."offline_data/alignments/".$user_id;
	if ($walk_valid_moved == false)
		exec("mv ".$path_alog_prefix."_walk_alog.txt ".$path_alog_prefix."_walk_alog.invalid.txt");
	if ($deliver_valid_moved == false)
		exec("mv ".$path_alog_prefix."_deliver_alog.txt ".$path_alog_prefix."_deliver_alog.invalid.txt");

	//don't pay user maybe
	//echo "validation error<br/>".$validation_response."<br/>".$target_validation; #DEBUG
	//die("Sorry; you failed to complete enough tasks correctly to warrant payment.");
}

//write MTurk validation to output
$mturk_code = $user_id."_".substr(sha1("rlg_salted_hash".$user_id),0,13);
echo "<p>Thank you for your participation!</p><p>Copy the code below, return to Mechanical Turk, and enter it to receive payment:<br/>".$mturk_code."</p>";

?>