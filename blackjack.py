import rospy
from std_msgs.msg import String
from collections import deque
from blackjack_core import get_card_value, calculate_hand  # import your reusable functions

class BlackjackRobotPlayer:
    def __init__(self):
        rospy.init_node('blackjack_player')
        self.card_sub = rospy.Subscriber('/detected_card_text', String, self.card_callback)
        self.emotion_sub = rospy.Subscriber('/player_emotion', String, self.emotion_callback)
        self.move_pub = rospy.Publisher('/robot_move', String, queue_size=10)

        self.player_hand = []
        self.dealer_hand = []
        self.round_active = False
        self.emotion = "neutral"
        self.previous_emotions = deque(maxlen=5)

        rospy.loginfo("Blackjack robot player ready.")
        rospy.spin()

    def emotion_callback(self, msg):
        self.emotion = msg.data.lower()
        self.previous_emotions.append(self.emotion)

    def card_callback(self, msg):
        card_text = msg.data.strip().title()  # Example: "10 Of Clubs"
        if not self.round_active:
            self.start_new_round()

        if card_text not in [c['text'] for c in self.player_hand]:
            self.player_hand.append({'text': card_text, 'value': get_card_value(card_text)})
            rospy.loginfo(f"Card added: {card_text}")
            self.evaluate_player_hand()

    def evaluate_player_hand(self):
        total = sum(card['value'] for card in self.player_hand)
        rospy.loginfo(f"Player hand total: {total}")

        threshold = self.get_emotion_threshold()
        if total < threshold:
            self.move_pub.publish("hit")
        elif total <= 21:
            self.move_pub.publish("stand")
            self.play_dealer()
        else:
            self.move_pub.publish("bust")
            self.end_round()

    def get_emotion_threshold(self):
        """
        Determines hit/stand threshold based on emotion.
        """
        emotion = self.emotion
        if emotion == 'sad':
            return 19  # Robot plays aggressively
        elif emotion == 'happy':
            return 16  # Robot takes more risk
        return 17

    def play_dealer(self):
        # Simulated dealer logic (replace with real dealer node if you have one)
        self.dealer_hand = self.simulate_dealer()
        player_total = sum(card['value'] for card in self.player_hand)
        dealer_total = sum(card['value'] for card in self.dealer_hand)

        rospy.loginfo(f"Dealer total: {dealer_total}")
        result = self.compare_totals(player_total, dealer_total)
        self.move_pub.publish(result)
        self.end_round()

    def simulate_dealer(self):
        # Dummy logic — replace with real card scanning or rules
        import random
        dealer = []
        total = 0
        while total < 17:
            value = random.randint(2, 11)
            total += value
            dealer.append({'value': value})
        return dealer

    def compare_totals(self, player, dealer):
        if dealer > 21 or player > dealer:
            return "win"
        elif player < dealer:
            return "lose"
        else:
            return "tie"

    def start_new_round(self):
        self.player_hand.clear()
        self.dealer_hand.clear()
        self.round_active = True
        rospy.loginfo("Starting new round.")

    def end_round(self):
        rospy.sleep(2)
        self.round_active = False
        rospy.loginfo("Round complete.\n---\n")

if __name__ == '__main__':
    try:
        BlackjackRobotPlayer()
    except rospy.ROSInterruptException:
        pass
