import httpx

# The URL for the wayfinder service
WAYFINDER_URL = "https://fe00-34-73-69-124.ngrok-free.app/wayfinder"

def initialize_wayfinder():
    """
    Initializes the wayfinder system by sending a one-time setup request.
    Sends a payload with source and destination information.
    """
    # The payload for the initialization request
    payload = {
        "action": "initialize",  # Action type for the request
        "source": "room436",  # Starting point for navigation
        "destination": "room424",  # Target destination
        "useClockDirections": False,  # Whether to use clock directions
        "useLandmarks": False  # Whether to use landmarks for navigation
    }
    
    print(f"Sending initialization request to {WAYFINDER_URL}...")
    print(f"Payload: {payload}")
    
    try:
        # Use a synchronous client for this one-off script
        with httpx.Client() as client:
            response = client.post(WAYFINDER_URL, json=payload)
            response.raise_for_status()  # Raise an exception for bad status codes (like 404 or 500)
            print("\nSuccessfully initialized wayfinder!")
            print(f"Response: {response.json()}")
            return response.json()
    except httpx.HTTPStatusError as e:
        # Handle HTTP status errors (e.g., 404, 500)
        print(f"\nError initializing wayfinder: {e}")
        print(f"Response content: {e.response.text}")
        return {"error": str(e)}
    except httpx.RequestError as e:
        # Handle request errors (e.g., connection issues)
        print(f"\nAn error occurred while requesting {e.request.url!r}.")
        print(f"Error details: {e}")
        return {"error": str(e)}

if __name__ == "__main__":
    # Entry point for the script
    initialize_wayfinder()
