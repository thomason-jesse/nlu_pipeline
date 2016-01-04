<?php
    
    $return_value = NULL;

    if( !isset($_POST['functionname']) ) { $return_value = ['No function name!']; }

    //if( !isset($_POST['arguments']) ) { $aResult['error'] = 'No function arguments!'; }

    if ($return_value == NULL) {

        switch($_POST['functionname']) {
            //case 'create_socket':
               //$return_value = create_socket();
               //break;
               
            case 'send':
               if( !isset($_POST['arg1'])  || !isset($_POST['arg2'])) {
                   $return_value = ['Incorrect arguments!'];
               }
               else {
                   $return_value = send($_POST['arg1'], $_POST['arg2']);
               }
               break;
               
            case 'receive':
               if( !isset($_POST['arg1']) ) {
                   $return_value = ['Error in arguments!'];
               }
               else {
                   $return_value = receive($_POST['arg1']);
               }
               break;   
               
            //case 'close_socket':
               //if( !isset($_POST['arg1']) ) {
                   //$return_value = ['Error in arguments!'];
               //}
               //else {
                   //$return_value = close_socket($_POST['arg1']);
               //}
               //break;   

            default:
               $return_value = ['Not found function '.$_POST['functionname'].'!'];
               break;
        }
    }
    
    function create_socket() {
        $port = 9999;    
        $address = '127.0.0.1';
        $socket = socket_create(AF_INET, SOCK_STREAM, SOL_TCP);
        if ($socket === false) {
            //$alert_str = "socket_create() failed. Error: ".socket_strerror(socket_last_error()); 
            //phpAlert($alert_str);
            //return ["socket_create() failed. Error: ".socket_strerror(socket_last_error())]; 
            return NULL;
        } 
        $result = socket_connect($socket, $address, $port);
        if ($result === false) {
            //$alert_str = "socket_connect() failed.\nReason: ($result) " . socket_strerror(socket_last_error($socket)) . "\n";
            //phpAlert($alert_str);
            //return ["socket_connect() failed.\nReason: ($result) " . socket_strerror(socket_last_error($socket)) . "\n"];
            return NULL;
        }
        //$json_socket = json_encode($socket);
        //return [NULL, $json_socket];
        return $socket;
    }

    function send($message, $json_socket) {
        try {
            $socket = NULL;
            if ($json_socket == NULL || json_decode($json_socket) == NULL) {
                $socket = create_socket();
                if ($socket == NULL) {
                    return ['Could not create socket.'];
                }          
            } else {
                $socket = json_decode($json_socket);
            }
            socket_write($socket, $message, strlen($message));
            return ['', '', json_encode($socket)];
        } catch (Exception $e) {
            return [$e->getMessage()];
        }
    }

    function receive($json_socket) {
        try {
            $socket = NULL;
            if ($json_socket == NULL || json_decode($json_socket) == NULL) {
                $socket = create_socket();
                if ($socket == NULL) {
                    return ['Could not create socket.'];
                }          
            } else {
                $socket = json_decode($json_socket);
            }
            $message = socket_read($socket, 2048);
            return ['', $message, json_encode($socket)];
        } catch (Exception $e) {
            return [$e->getMessage()];
        }
    }

    function close_socket($json_socket) {
        try {
            $socket = json_decode($json_socket);
            socket_close($socket);
            return ['',''];
        } catch (Exception $e) {
            return [$e->getMessage()];
        }
    }
    
    echo json_encode($return_value);
?>
