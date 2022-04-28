Once the claiberated camera is placed at an overhead position we can perfom automated pick and place operation.

Note that for the rest of this we will assume the "table" to be our fixed or world coordinte system.
First step is to obtain the transformation between the tabel and the camera

To do this place the April tag marker on the table and  record the estimated transfrom between the fixed camera and the marker. One can easily steimate the required transform between the table and the camera based in the above recorded transfrom.

We already know the trandfrom between the tabel and the robot base. 

Create a TF broadcastor to publish both of the above transfroms continously. 

Next step is to write the code for the oick and place operation itself. Follow this guide for the generic pick and place tutorial code where the operation is done to a pre recorded pose

Now we can modify the above code to bring the robot close to the april tag 