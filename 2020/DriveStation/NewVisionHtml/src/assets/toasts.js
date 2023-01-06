function displaytoast(message) {
  console.info(message)
  // Get the snackbar DIV
  const toast = document.getElementById("toast");

  // Add the "show" class to DIV
  toast.innerText = message

  toast.className = "show";
  // After 3 seconds, remove the show class from DIV
  setTimeout(function(){ toast.className = toast.className.replace("show", "");}, 5000);
}
// displaytoast('hello', 3)
// displaytoast('hello', 3)
