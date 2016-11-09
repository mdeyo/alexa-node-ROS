var numbers = {
  '.': 'point',
  '-': 'negative',
  0: 'zero',
  1: 'one',
  2: 'two',
  3: 'three',
  4: 'four',
  5: 'five',
  6: 'six',
  7: 'seven',
  8: 'eight',
  9: 'nine',
  10: 'ten',
  11: 'eleven',
  12: 'twelve',
  13: 'thirteen',
  14: 'fourteen',
  15: 'fifteen',
  16: 'sixteen',
  17: 'seventeen',
  18: 'eighteen',
  19: 'nineteen',
  20: 'twenty',
  30: 'thirty',
  40: 'forty',
  50: 'fifty',
  60: 'sixty',
  70: 'seventy',
  80: 'eighty',
  90: 'ninety'
};

// http://en.wikipedia.org/wiki/English_numerals#Cardinal_numbers
var helpers = {};
// Store the helpers in the power of tens
helpers[2]   = 'hundred';
helpers[3]   = 'thousand';
helpers[6]   = 'million';
helpers[9]   = 'billion';
helpers[12]  = 'trillion';
helpers[15]  = 'quadrillion';
helpers[18]  = 'quintillion';
helpers[21]  = 'sextillion';
helpers[24]  = 'septillion';
helpers[27]  = 'octillion';
helpers[30]  = 'nonillion';
helpers[33]  = 'decillion';
helpers[36]  = 'undecillion';
helpers[39]  = 'duodecillion';
helpers[42]  = 'tredecillion';
helpers[45]  = 'quattuordecillion';
helpers[48]  = 'quindecillion';
helpers[51]  = 'sexdecillion';
helpers[54]  = 'septendecillion';
helpers[57]  = 'octodecillion';
helpers[60]  = 'novemdecillion';
helpers[63]  = 'vigintillion';
helpers[100] = 'googol';
helpers[303] = 'centillion';

// Make a hash of the numbers and helper numbers reversed
// E.g. The key as the word and value as the number
var numbersMap = {};
numbersMap.nil     = 0;
numbersMap.naught  = 0;
numbersMap.period  = '.';
numbersMap.decimal = '.';

Object.keys(numbers).forEach(function (num) {
  numbersMap[numbers[num]] = isNaN(+num) ? num : +num;
});

Object.keys(helpers).forEach(function (num) {
  numbersMap[helpers[num]] = isNaN(+num) ? num : Math.pow(10, +num);
});

/**
 * Returns the number of significant figures for the number
 * @param  {number} num
 * @return {number}
 */
var intervals = function (num) {
  var match;
  if ((match = ('' + num).match(/e\+(\d+)/))) {
    return match[1];
  }

  return ('' + num).length - 1;
};

/**
 * Accepts both a string and number type - and return the opposite
 * @param  {string|number} num
 * @return {string|number}
 */
var numberWords = module.exports = function (num) {
  if (typeof num === 'string') {
    return numberWords.parse(num);
  }
  if (typeof num === 'number') {
    return numberWords.stringify(num);
  }
  throw new Error('Number words can handle handle numbers and/or strings');
};

/**
 * Turn a number into a string representation
 * @param  {number} num
 * @return {string}
 */
numberWords.stringify = function (num) {
  var word = [],
      interval,
      remaining;

  num = isNaN(+num) ? num : +num;

  // Numbers are super buggy in JS over 10^20
  if (typeof num !== 'number') { return false; }
  // If the number is in the numbers object, we can quickly return
  if (numbers[num]) { return numbers[num]; }
  // If the number is a negative value
  if (num < 0) {
    return numbers['-'] + ' ' + numberWords.stringify(num * -1);
  }

  // Check if we have decimals
  if (num % 1) {
    word.push(numberWords.stringify(Math.floor(num)));
    word.push(numbers['.']);
    word = word.concat(('' + num).split('.')[1].split('').map(numberWords.stringify));
    return word.join(' ');
  }

  interval = intervals(num);
  // It's below one hundred, but greater than nine
  if (interval === 1) {
    word.push(numbers[Math.floor(num / 10) * 10] + '-' + numberWords.stringify(Math.floor(num % 10)));
  }
  // Simple check to find the closest full number helper
  while (interval > 3 && !helpers[interval]) {
    interval -= 1;
  }

  if (helpers[interval]) {
    remaining = Math.floor(num % Math.pow(10, interval));
    word.push(numberWords.stringify(Math.floor(num / Math.pow(10, interval))));
    word.push(helpers[interval] + (remaining > 99 ? ',' : ''));
    if (remaining) {
      if (remaining < 100) { word.push('and'); }
      word.push(numberWords.stringify(remaining));
    }
  }

  return word.join(' ');
};

/**
 * Turns a string representation of a number into a number type
 * @param  {string} num
 * @return {number}
 */
numberWords.parse = function (num) {
  if (typeof num !== 'string') { return false; }

  var modifier        = 1,
      largest         = 0,
      largestInterval = 0,
      zeros           = 0, // Keep track of the number of leading zeros in the decimal
      stack           = [];

  var totalStack = function () {
    var total = stack.reduceRight(function (memo, num, index, array) {
      if (num > array[index + 1]) {
        return memo * num;
      }
      return memo + num;
    }, 0);

    return total * largest;
  };

  var total = num.split(/\W+/g).map(function (num) {
    num = num.toLowerCase(); // Make life easier
    return numbersMap[num] != null ? numbersMap[num] : num;
  }).filter(function (num) {
    if (num === '-') {
      modifier = -1;
    }
    if (num === '.') {
      return true; // Decimal points are a special case
    }
    return isFinite(num); // Remove numbers we don't understand
  }).reduceRight(function (memo, num) {
    var interval = intervals(num),
        decimals,
        output;

    // Check the interval is smaller than the largest one, then create a stack
    if (typeof num === 'number' && interval < largestInterval) {
      if (!stack.length) { memo = memo - largest; }
      stack.push(num);
      return memo;
    }

    memo  = memo + totalStack();
    stack = []; // Reset the stack for more computations

    // If the number is a decimal, transform everything we were just working with
    if (num === '.') {
      decimals = zeros + ('' + memo).length;
      zeros    = 0;
      // Reset the largest intervals and stuff
      largest         = 0;
      largestInterval = 0;
      return memo * Math.pow(10, decimals * -1);
    }

    // Keep a count of zeros we encountered
    if (num === 0) {
      zeros += 1;
      return memo;
    }

    // Shove the number on the front if the intervals match and the number is a whole
    if (memo >= 1 && interval === largestInterval) {
      output = '' + memo;
      // Decrement the zeros count while adding zeros to the front of the number
      while (zeros && zeros--) {
        output = '0' + output;
      }
      return +(num + output);
    }

    // Store the largest number for future use
    largest         = num;
    largestInterval = intervals(largest);

    return (memo + num) * Math.pow(10, zeros);
  }, 0);

  return modifier * (total + totalStack());
};
