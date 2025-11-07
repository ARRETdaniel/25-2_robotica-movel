function sysCall_init()
end

function sysCall_vision(inData)
    local retVal={}
    retVal.trigger=false
    retVal.packedPackets={}
    simVision.sensorDepthMapToWorkImg(inData.handle)
    local trig,packedPacket=simVision.coordinatesFromWorkImg(inData.handle,{342,1},false) if trig then retVal.trigger=true end if packedPacket then retVal.packedPackets[#retVal.packedPackets+1]=packedPacket end
    simVision.workImgToSensorImg(inData.handle)
    return retVal
end
