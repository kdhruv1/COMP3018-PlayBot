import random

suits = ['Hearts', 'Diamonds', 'Clubs','Spades']
ranks = ['2','3','4','5','6','7','8','9','10','Jack','Queen','King','Ace']
deck = [{'rank': rank, 'suit': suit} for suit in suits for rank in ranks]

#card values s

def get_card_value(card):
    if card['rank'] in ['Jack', 'Queen', 'King']:
        return 10
    elif card['rank'] == 'Ace':
        return 11  #add 1 value later 
    else:
        return int(card['rank'])
    


# Calculate total hand value
def calculate_hand(hand):
    value = sum(get_card_value(card) for card in hand)
    aces = sum(1 for card in hand if card['rank'] == 'Ace')
    while value > 21 and aces:
        value -= 10  # Convert Ace from 11 to 1
        aces -= 1
    return value

# Deal a card
def deal(deck):
    card = random.choice(deck)
    deck.remove(card)
    return card

# Display hand
def display_hand(hand, owner="Player"):
    print(f"\n{owner}'s Hand:")
    for card in hand:
        print(f"  {card['rank']} of {card['suit']}")
    print(f"Total Value: {calculate_hand(hand)}")

# main game function
def play_blackjack():
    print("Welcome to Blackjack!")

    # Setup deck and hands
    current_deck = deck.copy()
    random.shuffle(current_deck)
    player_hand = [deal(current_deck), deal(current_deck)]
    dealer_hand = [deal(current_deck), deal(current_deck)]

    # Show hands
    display_hand(player_hand)
    print("\nDealer's Hand:")
    print(f"  {dealer_hand[0]['rank']} of {dealer_hand[0]['suit']}")
    print("  [Hidden Card]")

    # Player turn
    while calculate_hand(player_hand) < 21:
        move = input("\nDo you want to 'hit' or 'stand'? ").lower()
        if move == 'hit':
            player_hand.append(deal(current_deck))
            display_hand(player_hand)
        elif move == 'stand':
            break
        else:
            print("Invalid input.")

    player_total = calculate_hand(player_hand)
    if player_total > 21:
        print("\nYou busted! Dealer wins.")
        return

    # Dealer's turn
    print("\nDealer reveals hand:")
    display_hand(dealer_hand, "Dealer")
    while calculate_hand(dealer_hand) < 17:
        dealer_hand.append(deal(current_deck))
        display_hand(dealer_hand, "Dealer")

    dealer_total = calculate_hand(dealer_hand)

    # final results
    print("\n--- Final Results ---")
    print(f"Your total: {player_total}")
    print(f"Dealer total: {dealer_total}")

    if dealer_total > 21 or player_total > dealer_total:
        print("You win!")
    elif player_total < dealer_total:
        print("Dealer wins.")
    else:
        print("It's a tie!")

# robot to run the game etc
if __name__ == "__main__":
    play_blackjack()
