int kt(int d, int s){
    if((d == 178) && (s == 180)) return 1;
    else if((d == 177) && (s == 180)) return 2;
    else if((d == 178) && (s == 181)) return 2;
    
    else if((d == 180) && (s == 178)) return -1;
    else if((d == 180) && (s == 177)) return -2;
    else if((d == 181) && (s == 178)) return -2;
    
    else if((d == 359) && (s == 0)) return 1;
    else if((d == 358) && (s == 0)) return 2;
    else if((d == 359) && (s == 1)) return 2;
    
    else if((d == 0) && (s == 359)) return -1;
    
    else if((d == 1) && (s == 359)) return -2;
    else if((d == 0) && (s == 358)) return -2;

    if(abs(s-d) > 180) return 0;
    
    return s-d;
}

void doc(){   
    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        goc = ypr[0] * 180/PI; 
        gocraw = goc;
        gocxoay = goc;
        
        if(gocxoay < 0) gocxoay = 360 + gocxoay;
        if(gocxoay != bd){
            xoay += kt(bd, gocxoay);;
            bd = gocxoay;
        }
    }
}