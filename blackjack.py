import rospy
from std_msgs.msg import String
import random

suits = ['Hearts', 'Diamonds', 'Clubs','Spades']
ranks = ['2','3','4','5','6','7','8','9','10','Jack','Queen','King','Ace']
deck = [{'rank': r, 'suit': s} for s in suits for r in ranks]

emotion_state = 'neutral'
threshold = 17

def get_card_value(card):
    if card['rank'] in ['Jack', 'Queen', 'King']:
        return 10
    elif card['rank'] == 'Ace':
        return 11
    return int(card['rank'])

def calculate_hand(hand):
    value = sum(get_card_value(card) for card in hand)
    aces = sum(1 for c in hand if c['rank'] == 'Ace')
    while value > 21 and aces:
        value -= 10
        aces -= 1
    return value

def emotion_callback(msg):
    global emotion_state, threshold
    emotion_state = msg.data
    if emotion_state == 'sad':
        threshold = 19
    elif emotion_state == 'happy':
        threshold = 15
    else:
        threshold = 17

def play_round():
    local_deck = deck[:]
    random.shuffle(local_deck)
    player = [local_deck.pop(), local_deck.pop()]
    robot = [local_deck.pop(), local_deck.pop()]
    print("\nYou:", player)
    print("Robot:", robot)
    while calculate_hand(robot) < threshold:
        robot.append(local_deck.pop())
    return player, robot

def main():
    rospy.init_node('blackjack_node')
    rospy.Subscriber('/player_emotion', String, emotion_callback)

    while not rospy.is_shutdown():
        player, robot = play_round()
        pv, rv = calculate_hand(player), calculate_hand(robot)
        print(f"Final hands: You {pv}, Robot {rv}")
        if pv > 21 or (rv <= 21 and rv > pv):
            print("Robot wins!")
        elif rv > 21 or pv > rv:
            print("You win!")
        else:
            print("Tie!")
        input("Press Enter for next round...")

if __name__ == '__main__':
    main()
