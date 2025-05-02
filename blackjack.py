import random
import threading
import time
from camera_win import subscribe, camera_loop

# Build a new deck
suits = ['Hearts','Diamonds','Clubs','Spades']
ranks = ['2','3','4','5','6','7','8','9','10','Jack','Queen','King','Ace']
full_deck = [{'rank': r, 'suit': s} for s in suits for r in ranks]

# State
player_hand = []
robot_hand  = []
deck        = []
emotion     = 'neutral'
hit_thresh  = 17

def card_callback(data):
    """When camera_win publishes a card event."""
    text = data['text']
    rank, _, suit = text.partition(' of ')
    card = {'rank': rank, 'suit': suit}
    if card not in player_hand:
        player_hand.append(card)
        print(f">>> You scanned: {text}")

def emotion_callback(e):
    """Update emotion and threshold."""
    global emotion, hit_thresh
    emotion = e
    if emotion == 'sad':
        hit_thresh = 16
    elif emotion == 'happy':
        hit_thresh = 18
    else:
        hit_thresh = 17
    print(f">>> Emotion: {emotion}. Robot hit threshold set to {hit_thresh}")

def card_value(c):
    if c['rank'] in ['Jack','Queen','King']: return 10
    if c['rank']=='Ace': return 11
    return int(c['rank'])

def hand_value(hand):
    v = sum(card_value(c) for c in hand)
    aces = sum(1 for c in hand if c['rank']=='Ace')
    while v>21 and aces:
        v -= 10; aces -= 1
    return v

def play_round():
    global deck, player_hand, robot_hand
    deck = full_deck.copy()
    random.shuffle(deck)
    player_hand = []
    robot_hand  = [deck.pop(), deck.pop()]

    print("\n=== New Round ===")
    print(f"Robot initial hand: {[f\"{c['rank']} of {c['suit']}\" for c in robot_hand]}")

    # Wait up to 10s for you to scan your two cards
    start = time.time()
    while len(player_hand)<2 and time.time()-start<10:
        time.sleep(0.1)

    print(f"Your hand: {[f\"{c['rank']} of {c['suit']}\" for c in player_hand]}")
    print(f"Robot threshold = {hit_thresh}")

    # Robot hits until reaching threshold
    while hand_value(robot_hand) < hit_thresh:
        c = deck.pop()
        robot_hand.append(c)
        print(f"Robot hits and draws {c['rank']} of {c['suit']} → total {hand_value(robot_hand)}")
        time.sleep(0.5)

    print(f"Robot stands with {hand_value(robot_hand)}")

    # Dealer play (simple auto 17 rule)
    dealer = []
    while hand_value(dealer)<17:
        c = deck.pop()
        dealer.append(c)
    print(f"Dealer hand: {[f\"{c['rank']} of {c['suit']}\" for c in dealer]} → {hand_value(dealer)}")

    # Final compare
    pv = hand_value(player_hand)
    rv = hand_value(robot_hand)
    dv = hand_value(dealer)
    print(f"Totals → You: {pv}, Robot: {rv}, Dealer: {dv}")
    if pv>21 or (rv<=21 and rv>pv):
        print("*** Robot wins! ***")
    elif rv>21 or pv>rv:
        print("*** You win! ***")
    else:
        print("*** It's a tie! ***")

def main():
    # Subscribe first
    subscribe('card',    card_callback)
    subscribe('emotion', emotion_callback)

    # Start camera in background thread
    cam_t = threading.Thread(target=camera_loop, daemon=True)
    cam_t.start()

    # Game loop
    try:
        while True:
            play_round()
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == '__main__':
    main()
