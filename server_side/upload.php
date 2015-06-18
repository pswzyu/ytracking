<?php

    $uploaded_type = $_FILES['uploaded']['type'];

    $target = "/var/www/html-glhf/epic_car/";
    $target = $target . basename( $_FILES['uploaded']['name']) ;
    $ok=1;

    //echo "target: ";
    //echo $target;
    
    $num_car = $_POST["count"];
    if ($num_car != "null")
    {
        $f = fopen("count.html", "w+");
        fwrite($f, $num_car);
        fclose($f);
    }
    
    print_r($_POST);

    //This is our limit file type condition
    if ($uploaded_type =="text/xml"){
            //echo "We have an xml file!\r\n";
    }

    //Here we check that $ok was not set to 0 by an error
    //If everything is ok we try to upload it
    if ($ok==0){

            //echo "Sorry your file was not uploaded";

    } else {

            //echo "Looking good!";

            if(move_uploaded_file($_FILES['uploaded']['tmp_name'], $target)){
                    //echo "The file successfully ". basename( $_FILES['uploadedfile']['name']). " has been uploaded";
            } else {
                    //echo "Sorry, there was a problem uploading your file.";
            }
    }
?>