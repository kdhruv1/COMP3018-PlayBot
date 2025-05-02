#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from collections import deque

class BlackjackPlayer:
    def __init__(self):
        rospy.init_node('blackjack_player')

        # State
        self.hand = []
        self.known_cards = set()
        self.player_emotion = 'neutral'
        self.win_streak = 0
        self.last_game_result = None

        # Subscribers
        rospy.Subscriber('/detected_card_pose', PoseStamped, self.card_callback, queue_size=1)
        rospy.Subscriber('/player_emotion', String, self.emotion_callback, queue_size=1)

        # Publisher for game decisions (optional)
        self.move_pub = rospy.Publisher('/robot_blackjack_move', String, queue_size=1)

        rospy.loginfo("Blackjack robot player node ready.")
        rospy.spin()

    def emotion_callback(self, msg):
        self.player_emotion = msg.data.lower()
        rospy.loginfo(f"[Emotion] Player seems {self.player_emotion}")

    def card_callback(self, msg):
        # Simulate card recognition from position (you should attach card label in a real system)
        card_id = f"{msg.pose.position.x:.2f}{msg.pose.position.y:.2f}{msg.pose.position.z:.2f}"
        if card_id in self.known_cards:
            return  # avoid duplicate detection
        self.known_cards.add(card_id)

        # Simulated card (in real usage, map from position or topic label)
        import random
        card = random.choice([
            '2 of hearts', '3 of diamonds', '10 of clubs',
            'J of spades', 'A of hearts', 'K of clubs'
        ])
        self.hand.append(card)
        rospy.loginfo(f"[Card] Added: {card}")
        self.evaluate_hand()

    def evaluate_hand(self):
        value = self.calculate_hand_value()
        rospy.loginfo(f"[Hand] Current hand: {self.hand} => Total: {value}")

        if value > 21:
            rospy.loginfo("[Result] BUST!")
            self.last_game_result = 'loss'
            self.hand.clear()
            self.known_cards.clear()
            return

        threshold = self.dynamic_threshold()
        if value < threshold:
            self.take_action('hit')
        else:
            self.take_action('stand')

    def dynamic_threshold(self):
        base = 17
        if self.player_emotion == 'sad':
            base -= 1  # play more aggressively
        elif self.player_emotion == 'happy' and self.win_streak > 1:
            base += 1  # play more conservatively
        rospy.loginfo(f"[AI] Adjusted hit threshold: {base}")
        return base

    def take_action(self, action):
        rospy.loginfo(f"[Move] Robot chooses to: {action.upper()}")
        self.move_pub.publish(action)

        if action == 'stand':
            # Simulate outcome (replace with real game logic)
            import random
            result = random.choice(['win', 'loss'])
            self.last_game_result = result
            if result == 'win':
                self.win_streak += 1
            else:
                self.win_streak = 0
            rospy.loginfo(f"[Result] Robot {result.upper()}!")
            self.hand.clear()
            self.known_cards.clear()

    def calculate_hand_value(self):
        total = 0
        aces = 0
        for card in self.hand:
            rank = card.split()[0]
            if rank in ['J', 'Q', 'K']:
                total += 10
            elif rank == 'A':
                aces += 1
                total += 11
            else:
                total += int(rank)

        while total > 21 and aces:
            total -= 10
            aces -= 1
        return total

if __name__ == '__main__':
    try:
        BlackjackPlayer()
    except rospy.ROSInterruptException:
        pass
