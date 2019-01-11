ros::init( argc, argv, nodeName);
ros::NodeHandle n;

ros::publisher  topicName = n.advertise<bitten::messageTypeName> (topicName , bufferSize);
ros::subscriber topicName = n.subscribe<bitten::messageTypeName> (topicName , bufferSize, &callbackFunctionName);

ros::Rate loop_rate(50); 

while(ros::ok())
{	
	// do stuff
	
	if (transmitReady == true)
	{
		topicName.publish(messageName);
		transmitReady = false;
	}
	
	ros::spinOnce();
	loop_rate.sleep();
}



void callbackFunctionName(const bitten::messageTypeName::constPtr& callbackMessage)
{
	// do stuff
	
	transmitReady = true;
}



