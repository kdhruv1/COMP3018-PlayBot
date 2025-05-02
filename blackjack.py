import random
import time
from camera_win import run_camera

deck = [f"{r} of {s}" for s in ['Hearts', 'Diamonds', 'Clubs', 'Spades']
                       for r in ['2','3','4','5','6','7','8','9','10','Jack','Queen','King','Ace']]

values = {
    **{str(i): i for i in range(2, 11)},
    'Jack': 10, 'Queen': 10, 'King': 10, 'Ace': 11
}

def card_value(card):
    rank = card.split(' ')[0]
    return values[rank]

def hand_value(hand):
    total = sum(card_value(c) for c in hand)
    aces = sum(1 for c in hand if c.startswith('Ace'))
    while total > 21 and aces:
        total -= 10
        aces -= 1
    return total

# State
player_hand = []
robot_hand = []
emotion = "neutral"
player_wins = 0
robot_wins = 0
current_qrs = set()

def on_qr(text, pos):
    if text not in current_qrs and text in deck:
        print(f"[SCAN] You scanned: {text}")
        player_hand.append(text)
        current_qrs.add(text)

def on_emotion(e):
    global emotion
    print(f"[EMOTION] {e}")
    emotion = e

def dealer_strategy(threshold):
    while hand_value(robot_hand) < threshold:
        card = random.choice(deck)
        robot_hand.append(card)
        print(f"[ROBOT] Hits: {card}")

def play_round():
    global player_hand, robot_hand, current_qrs, player_wins, robot_wins

    player_hand = []
    robot_hand = []
    current_qrs.clear()

    print("\n[ROUND] Please scan 2 cards.")
    start = time.time()
    while len(player_hand) < 2 and time.time() - start < 20:
        time.sleep(0.2)

    print(f"Your hand: {player_hand} (value = {hand_value(player_hand)})")

    # Adjust robot strategy based on emotion & scores
    if emotion == 'sad' and player_wins < robot_wins:
        threshold = 19 # Easier on sad player
    elif emotion == 'happy' and player_wins > robot_wins:
        threshold = 17 # Robot plays stronger
    else:
        threshold = 16 # Normal

    robot_hand.append(random.choice(deck))
    robot_hand.append(random.choice(deck))
    print(f"[ROBOT] Starting hand: {robot_hand}")
    dealer_strategy(threshold)

    pv = hand_value(player_hand)
    rv = hand_value(robot_hand)
    print(f"Your total: {pv}, Robot total: {rv}")

    if pv > 21:
        print("You busted. Robot wins.")
        robot_wins += 1
    elif rv > 21 or pv > rv:
        print("You win!")
        player_wins += 1
    elif pv < rv:
        print("Robot wins.")
        robot_wins += 1
    else:
        print("It's a tie.")

def main():
    import threading
    t = threading.Thread(target=run_camera, args=(on_qr, on_emotion), daemon=True)
    t.start()

    while True:
        input("\nPress Enter to play a round...")
        play_round()

if __name__ == "__main__":
    main()
