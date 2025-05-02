import rospy
from std_msgs.msg import String
import random

# build full deck
suits = ['Hearts','Diamonds','Clubs','Spades']
ranks = ['2','3','4','5','6','7','8','9','10','Jack','Queen','King','Ace']
full_deck = [{'rank': r, 'suit': s} for s in suits for r in ranks]

# state
player_hand = []
robot_hand  = []
deck         = []
emotion_state = 'neutral'
hit_threshold = 17

def get_card_value(card):
    if card['rank'] in ['Jack','Queen','King']:
        return 10
    if card['rank'] == 'Ace':
        return 11
    return int(card['rank'])

def calculate_hand(hand):
    total = sum(get_card_value(c) for c in hand)
    aces  = sum(1 for c in hand if c['rank']=='Ace')
    while total>21 and aces:
        total -= 10
        aces  -= 1
    return total

def emotion_callback(msg):
    global emotion_state, hit_threshold
    emotion_state = msg.data.lower()
    if emotion_state == 'sad':
        hit_threshold = 16
    elif emotion_state == 'happy':
        hit_threshold = 18
    else:
        hit_threshold = 17
    rospy.loginfo("Emotion set to %s, threshold=%d", emotion_state, hit_threshold)

def card_callback(msg):
    """Called when a new card text arrives from camera.py"""
    text = msg.data.strip()
    rank, _, suit = text.partition(' of ')
    card = {'rank': rank, 'suit': suit}
    if card not in player_hand:
        player_hand.append(card)
        rospy.loginfo("You drew: %s", text)

def deal_random(hand, count=1):
    for _ in range(count):
        c = random.choice(deck)
        deck.remove(c)
        hand.append(c)

def play_round():
    global deck, player_hand, robot_hand
    deck = full_deck.copy()
    random.shuffle(deck)
    player_hand = []
    robot_hand  = []

    # deal initial two cards
    deal_random(player_hand, 2)
    deal_random(robot_hand, 2)

    rospy.loginfo("Starting round. Robot has %d and %d",
                  get_card_value(robot_hand[0]), get_card_value(robot_hand[1]))

    # robot (auto) plays
    while calculate_hand(robot_hand) < hit_threshold:
        c = deck.pop()
        robot_hand.append(c)
        rospy.loginfo("Robot hits and draws %s of %s", c['rank'], c['suit'])
        rospy.sleep(0.5)

    rospy.loginfo("Robot stands with total %d", calculate_hand(robot_hand))

    # dealer
    dealer = []
    while sum(get_card_value(c) for c in dealer) < 17:
        c = deck.pop()
        dealer.append(c)

    # evaluate
    p_total = calculate_hand(player_hand)
    r_total = calculate_hand(robot_hand)
    d_total = calculate_hand(dealer)
    rospy.loginfo("Final totals → You: %d  Robot: %d  Dealer: %d", p_total, r_total, d_total)

    if p_total>21 or (r_total<=21 and r_total>p_total):
        rospy.loginfo("Robot wins!")
    elif r_total>21 or p_total>r_total:
        rospy.loginfo("You win!")
    else:
        rospy.loginfo("Tie!")

def main():
    rospy.init_node('blackjack_node')
    rospy.Subscriber('/player_emotion', String, emotion_callback)
    rospy.Subscriber('/detected_card_text', String, card_callback)

    rate = rospy.Rate(0.1)  # 1 round every 10 seconds
    while not rospy.is_shutdown():
        play_round()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
