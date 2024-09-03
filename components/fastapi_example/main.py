from fastapi import FastAPI
import uvicorn
from colorama import Fore
from fastapi.responses import HTMLResponse

app = FastAPI()


html = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Button Click Example</title>
    <script>
        function changeTextAndRedirect() {
            document.getElementById("myButton").innerHTML = "You really click every button you see? You're an IT's worst nightmare";
            setTimeout(function() {
                window.location.href = "https://www.youtube.com/watch?v=2qBlE2-WL60"; // Change to your desired URL
            }, 3000);
        }
    </script>
</head>
<body>
    <button id="myButton" onclick="changeTextAndRedirect()">Click me</button>
</body>
</html>
"""


@app.get("/", response_class=HTMLResponse)
async def root():
    return html

# Run the FastAPI app
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)