<html>
 <head>
  <title>PHP Test</title>
 </head>
 <body>

<?php

    function phpAlert($msg) {
        echo '<script type="text/javascript">alert("' . $msg . '")</script>';
    }

    function createSocket() {
        $port = 9999;    
        $address = '128.83.120.240';
        $socket = socket_create(AF_INET, SOCK_STREAM, SOL_TCP);
        if ($socket === false) {
            phpAlert('Failed to create socket: '.socket_strerror(socket_last_error()));
            die();
            return NULL;
        } else {
            $result = socket_connect($socket, $address, $port);
            if ($result === false) {
                phpAlert('Failed to connect to server: '.socket_strerror(socket_last_error($socket)));
                die();
                return NULL;
            } else {
                return $socket;    
            }
        }        
    }
    
    function send($message, $socket) {
        try {
            //socket_write($socket, $message, strlen($message)+1);
            socket_send($socket, $message, strlen($message), 0);
        } catch (Exception $e) {
            phpAlert($e->getMessage());
        }
    }

    function receive($socket) {
        try {
            //$message = socket_read($socket, 2048);
            $bytes = socket_recv($socket, $message, 2048, 0);
            if ($message === false) {
                phpAlert(socket_strerror(socket_last_error($socket)));
            }
            return $message;
        } catch (Exception $e) {
            phpAlert($e->getMessage());
        }
    }

    function close_socket($socket) {
        try {
            socket_close($socket);
        } catch (Exception $e) {
            phpAlert($e->getMessage());
        }
    }

    // Create socket
    echo 'Going to create socket<br/>';
    $socket = createSocket();
    if ($socket != NULL) {
        send("Hello", $socket);
        $msg = receive($socket);
        echo 'msg='.$msg.'<br/>';
        echo socket_strerror(socket_last_error($socket)).'<br/>';
        //send("Walk to bob\'s office", $socket);
        //echo socket_strerror(socket_last_error($socket)).'<br/>';
        $msg = receive($socket);
        echo 'msg='.$msg.'<br/>';
    }

?>
</body>
</html>
