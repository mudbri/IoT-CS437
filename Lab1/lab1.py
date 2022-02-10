import picar_4wd as fc
from picar_4wd import Ultrasonic

speed = 5
ANGLE_RANGE = 90
STEP = 18
max_angle = ANGLE_RANGE/2
min_angle = -ANGLE_RANGE/2

def main():
    current_angle = 0
    us_step = STEP
    while True:
        #iterates through angles
        current_angle += us_step
        if current_angle >= max_angle:
            current_angle = max_angle
            us_step = -STEP
        elif current_angle <= min_angle:
            current_angle = min_angle
            us_step = STEP
        #gets distance at current angle and sets servo to the angle
        dis_val =  fc.get_distance_at(current_angle)
        print(str(current_angle) + ": " + str(dis_val))
        #checks if car is close to object
        if dis_val < 20:
            #stops car
            count_stop = 0
            while count_stop < 100:
                fc.stop()
                count_stop += 1
            print("stop complete")
            if (count_stop == 100):
                #moves car backward
                count_back = 0
                while count_back < 300:
                    fc.backward(speed)
                    count_back += 1
                print("backword complete")
                if count_back == 300:
                    #turns car
                    count_turn = 0
                    while count_turn < 300:
                        fc.turn_right(speed)
                        count_turn += 1
                    print("turn complete")
        else:
            fc.forward(speed)

if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()