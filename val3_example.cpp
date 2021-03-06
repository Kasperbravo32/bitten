switch rosGenericMsg.header.nMsgType
    case 11
      l_mTrajPt.mDesc.vel= limit((rosTrajPtMsg.jointTrajPt.nVelocity * 100), 0, 100)
    break
    case 14
      l_mTrajPt.mDesc.vel= limit((rosTrajPtFMsg.jointTrajPtFull.nVelocities[0] * 100), 0, 100)
      l_mTrajPt.mDesc.accel=rosTrajPtFMsg.jointTrajPtFull.nAccelerations[0]
      l_mTrajPt.mDesc.decel=l_mTrajPt.mDesc.accel
    break
  endSwitch

  if (l_mTrajPt.nSequence==-4)
    bStopNow=true
  else
    if (bOverwriteVel == true)
      l_mTrajPt.mDesc.vel = getMonitorSpeed()
      l_mTrajPt.mDesc.accel = 9999
      l_mTrajPt.mDesc.decel = 9999
    endIf
    if (isInRange(l_mTrajPt.jJointRx) and l_mTrajPt.mDesc.vel > 0)
      call libQueueFuncs:push(qTrajPtBuffer,l_mTrajPt,l_bOk)
    else
      l_bOk = false
    endIf
	
	
