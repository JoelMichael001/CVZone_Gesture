from cvzone.HandTrackingModule import HandDetector
import cv2
from pymodbus.client import ModbusTcpClient

PLC_PORT = 502
PLC_IP = '192.168.0.190'
client = ModbusTcpClient(host=PLC_IP, port=PLC_PORT)

# Attempt to connect to the PLC
try:
    connection_status = client.connect()
    if connection_status:
        print(f"Connected to {PLC_IP}:{PLC_PORT} successfully")
    else:
        print("Could not connect to the PLC")
except Exception as e:
    print(f"Connection Error: {e}")

# Initialize Hand Detector
detector = HandDetector(mode=False, maxHands=1, detectionCon=0.7, minTrackCon=0.7)

# Camera Setup
cap = cv2.VideoCapture(0)



# Function to map distance to register value
def map_distance_to_value(distance, min_distance=20, max_distance=200):
    distance = max(min_distance, min(max_distance, distance))
    return int(((distance - min_distance) / (max_distance - min_distance)) * (MAX_VALUE - MIN_VALUE) + MIN_VALUE)

# Main Loop
while True:
    REGISTER_ADDRESS = 10
    MIN_VALUE = 5000
    MAX_VALUE = 32000
    success, img = cap.read()

    if not success:
        continue

    hands, img = detector.findHands(img, draw=True, flipType=True)

    if hands:
        hand1 = hands[0]
        fingers = detector.fingersUp(hand1)
        finger_count = fingers.count(1)

        # Tilt detection based on wrist and middle finger positions
          # Middle finger MCP joint

        if finger_count > 0:
            lmList = hand1['lmList']
            wrist = lmList[0]  # Wrist landmark
            middle_finger_mcp = lmList[9]

            thumb_tip = lmList[4]
            index_tip = lmList[8]

            # Calculate the distance between thumb and index finger
            distance = ((thumb_tip[0] - index_tip[0]) ** 2 + (thumb_tip[1] - index_tip[1]) ** 2) ** 0.5

            # Map distance to register value
            register_value = map_distance_to_value(distance)

            # Write to PLC
            client.write_coil(0, True)
            client.write_register(REGISTER_ADDRESS, register_value)
            print(f"Register {REGISTER_ADDRESS} Value: {register_value}")

            if middle_finger_mcp[0] > wrist[0] + 20:  # Tilted to the right
                client.write_coil(0, True)
                client.write_coil(1, False)
                client.write_coil(2, True)
                client.write_coil(4, False)
                print("Motor: Forward")
            elif middle_finger_mcp[0] < wrist[0] - 20:  # Tilted to the left
                client.write_coil(0, True)
                client.write_coil(1, True)
                client.write_coil(2, False)
                client.write_coil(4, False)
                print("Motor: Reverse")
        elif finger_count ==0:
            client.write_coil(0, False)
            client.write_coil(1, False)
            client.write_coil(2, False)
            client.write_coil(4, True)
            print("Motor: OFF")
        else:
            client.write_coil(0, False)
            client.write_coil(1, False)
            client.write_coil(2, False)
            client.write_coil(4, True)
            print("Motor: OFF")


        # Thumb tip (4) and index finger tip (8)


    # Display image
    cv2.imshow("Hand Gesture Control", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()