package org.firstinspires.ftc.teamcode;

/**
 * TragectoryGen is a class that returns a motion profile of a one dimension movement that can be used for the velocity control loop
 */
public class TrajectoryGen {


    public static void main(String args[]) {

        float[][] tra = fineIllCreateAGeneralGenClass(8, 10, 14f, 80f);
        for (int i = 0; i < tra.length; i++) {
            System.out.println(i / 1000f + ", " + tra[i][0] + ", " + tra[i][1]);
        }
    }

    public static float[][] fineIllCreateAGeneralGenClass(float maxVelocity, float distance, float maxAcceleration, float jerk) {
        float t1Time = maxAcceleration / jerk;
        float t2Time = (maxVelocity - (maxAcceleration * t1Time)) / maxAcceleration;
        float t1Displacement = (1 / 6f) * (float) Math.pow(t1Time, 3) * jerk;
        float t2Displacement = (1 / 2f) * (float) Math.pow(t2Time, 2) * maxAcceleration;
        float t3Displacement = t1Displacement;
        float rampDisplacement = (t1Displacement + t2Displacement + t3Displacement);
        float[][] tragectory;
        if (rampDisplacement > distance) {
            tragectory = okButwhatIfItsShort(maxVelocity, distance, maxAcceleration, jerk);
        } else {
            tragectory = S_Curve(maxVelocity, distance, maxAcceleration, jerk);
        }
        return tragectory;
    }

    public static float[][] okButwhatIfItsShort(float maxVelocity, float distance, float maxAcceleration, float jerk) {
        float velocity = 0;
        float position = 0;
        float acceleration = 0;
        float t1Time = maxAcceleration / jerk;
        float t2Time = (maxVelocity - (maxAcceleration * t1Time)) / maxAcceleration;
        float t1Displacement = (1 / 6f) * (float) Math.pow(t1Time, 3) * jerk;
        float t2Displacement = (1 / 2f) * (float) Math.pow(t2Time, 2) * maxAcceleration;
        float t3Displacement = t1Displacement;
        float rampTime = (t1Time + t2Time + t1Time);
        float rampDisplacement = (t1Displacement + t2Displacement + t3Displacement);
        float[][] tragectory = new float[(int) (rampTime * 1000 * 2) + 1][3];

        int t1 = 0;

        for (int i = 0; i < t1Time * 1000f; i++) {
            acceleration = jerk * t1 / 1000f;
            tragectory[t1][0] = acceleration;
            velocity = acceleration * (1f / 1000f) + velocity;
            tragectory[t1][1] = velocity;
            t1++;

        }
        for (int i = 0; i < t2Time * 1000f; i++) {
            tragectory[t1][0] = maxAcceleration;
            velocity = tragectory[t1][0] * 1 / 1000f + velocity;
            tragectory[t1][1] = velocity;
            t1++;
        }
        for (int i = 0; i < t1Time * 1000f; i++) {
            acceleration = acceleration - jerk * (1 / 1000f);
            tragectory[t1][0] = acceleration;
            velocity = acceleration * (1f / 1000f) + velocity;
            tragectory[t1][1] = velocity;
            t1++;
        }
        for (int i = 0; i < t1Time * 1000f; i++) {
            acceleration = jerk * i / 1000f;
            tragectory[t1][0] = acceleration;
            velocity = velocity - acceleration * (1f / 1000f);
            tragectory[t1][1] = velocity;
            t1++;

        }
        for (int i = 0; i < t2Time * 1000f; i++) {
            tragectory[t1][0] = maxAcceleration;
            velocity = velocity - maxAcceleration / 1000f;
            tragectory[t1][1] = velocity;
            t1++;
        }
        for (int i = 0; i < t1Time * 1000f - 1; i++) {
            acceleration = acceleration - jerk * (1 / 1000f);
            tragectory[t1][0] = acceleration;
            velocity = velocity - acceleration * (1f / 1000f);
            tragectory[t1][1] = velocity;
            t1++;

        }
        return tragectory;
    }

    private static float[][] S_Curve(float maxVelocity, float distance, float maxAcceleration, float jerk) {

        float velocity = 0;
        float position = 0;
        float acceleration = 0;
        float t1Time = maxAcceleration / jerk;
        float t2Time = (maxVelocity - (maxAcceleration * t1Time)) / maxAcceleration;
        float t1Displacement = (1 / 6f) * (float) Math.pow(t1Time, 3) * jerk;
        float t2Displacement = (1 / 2f) * (float) Math.pow(t2Time, 2) * maxAcceleration;
        float t3Displacement = t1Displacement;
        float rampTime = (t1Time + t2Time + t1Time);
        float rampDisplacement = (t1Displacement + t2Displacement + t3Displacement);
        float t4Displacement = distance - (rampDisplacement * 2);
        float t4Time = maxVelocity / t4Displacement;
        float[][] tragectory = new float[(int) (rampTime * 1000 + rampTime * 1000f + t4Time * 1000f) + 1][3];
        System.out.println(rampTime + " " + t4Time);
        int t1 = 0;
        for (int i = 0; i < t1Time * 1000f; i++) {
            acceleration = jerk * t1 / 1000f;
            tragectory[t1][0] = acceleration;
            velocity = acceleration * (1f / 1000f) + velocity;
            tragectory[t1][1] = velocity;
            t1++;

        }
        for (int i = 0; i < t2Time * 1000f; i++) {
            tragectory[t1][0] = maxAcceleration;
            velocity = tragectory[t1][0] * 1 / 1000f + velocity;
            tragectory[t1][1] = velocity;
            t1++;
        }
        for (int i = 0; i < t1Time * 1000f; i++) {
            acceleration = acceleration - jerk * (1 / 1000f);
            tragectory[t1][0] = acceleration;
            velocity = acceleration * (1f / 1000f) + velocity;
            tragectory[t1][1] = velocity;
            t1++;

        }
        for (int i = 0; i < t4Time * 1000f; i++) {
            tragectory[t1][0] = 0.0f;
            tragectory[t1][1] = maxVelocity;
            t1++;
        }
        for (int i = 0; i < t1Time * 1000f; i++) {
            acceleration = jerk * i / 1000f;
            tragectory[t1][0] = acceleration;
            velocity = velocity - acceleration * (1f / 1000f);
            tragectory[t1][1] = velocity;
            t1++;

        }
        for (int i = 0; i < t2Time * 1000f; i++) {
            tragectory[t1][0] = maxAcceleration;
            velocity = velocity - maxAcceleration / 1000f;
            tragectory[t1][1] = velocity;
            t1++;
        }
        for (int i = 0; i < t1Time * 1000f - 1; i++) {
            acceleration = acceleration - jerk * (1 / 1000f);
            tragectory[t1][0] = acceleration;
            velocity = velocity - acceleration * (1f / 1000f);
            tragectory[t1][1] = velocity;
            t1++;

        }


        return tragectory;
    }

    private static float[][] Trapazoid(float maxVelocity, float distance, float maxAcceleration) {
        float rampTime = maxVelocity / maxAcceleration;
        float rampDistance = rampTime * maxAcceleration / 2;
        float sustainDistance = distance - rampDistance * 2;

        float sustainTime = sustainDistance / maxVelocity;
        int timeToCompleteMili = (int) ((rampTime * 2 + sustainTime) * 1000) + 1;
        float[][] tragectory = new float[timeToCompleteMili][2];
        int t = 0;
        float velocity = 0;
        float position = 0;

        for (int i = 0; i < rampTime * 1000; i++) {
            velocity = (i * maxAcceleration) / 1000;
            position += velocity / 1000;
            tragectory[t][0] = velocity;
            tragectory[t][1] = position;
            t++;
        }
        for (int i = 0; i < sustainTime * 1000; i++) {
            velocity = 6;
            position += velocity / 1000;
            tragectory[t][0] = velocity;
            tragectory[t][1] = position;
            t++;

        }
        for (int i = 0; i < rampTime * 1000; i++) {
            velocity = velocity - (maxAcceleration / 1000);
            position += velocity / 1000;
            tragectory[t][0] = velocity;
            tragectory[t][1] = position;
            t++;
        }

        return tragectory;
    }

    public static float[] Triangle(float distance, float MaxAcceleration) {
        int timeToCompleteMili = (int) (distance * 2f / MaxAcceleration * 1000);

        float velocity = 0;
        float[] Tragectory = new float[timeToCompleteMili];
        for (int t = 0; t < (timeToCompleteMili) / 2; t++) {
            velocity = t * MaxAcceleration / 1000;
            Tragectory[t] = velocity;
        }
        int j = 0;
        for (int t = timeToCompleteMili / 2; t < (timeToCompleteMili); t++) {
            velocity = velocity - MaxAcceleration / 1000;
            Tragectory[t] = velocity;
            j++;
        }
        return Tragectory;
    }
}
