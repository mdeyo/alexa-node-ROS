/*global describe,it*/
var assert  = require('assert');
var numbers = require('./');

describe('number words', function () {
  it('should turn numbers into words', function () {
    assert.equal(numbers(0), 'zero');
    assert.equal(numbers(1), 'one');
    assert.equal(numbers(2), 'two');
    assert.equal(numbers(3), 'three');
    assert.equal(numbers(4), 'four');
    assert.equal(numbers(5), 'five');
    assert.equal(numbers(6), 'six');
    assert.equal(numbers(7), 'seven');
    assert.equal(numbers(8), 'eight');
    assert.equal(numbers(9), 'nine');
    assert.equal(numbers(10), 'ten');
    assert.equal(numbers(11), 'eleven');
    assert.equal(numbers(12), 'twelve');
    assert.equal(numbers(13), 'thirteen');
    assert.equal(numbers(14), 'fourteen');
    assert.equal(numbers(15), 'fifteen');
    assert.equal(numbers(16), 'sixteen');
    assert.equal(numbers(17), 'seventeen');
    assert.equal(numbers(18), 'eighteen');
    assert.equal(numbers(19), 'nineteen');
    assert.equal(numbers(20), 'twenty');
  });

  it('should handle tens', function () {
    assert.equal(numbers(29), 'twenty-nine');
    assert.equal(numbers(36), 'thirty-six');
    assert.equal(numbers(45), 'forty-five');
    assert.equal(numbers(51), 'fifty-one');
    assert.equal(numbers(63), 'sixty-three');
    assert.equal(numbers(78), 'seventy-eight');
    assert.equal(numbers(84), 'eighty-four');
    assert.equal(numbers(92), 'ninety-two');
  });

  it('should work normally with negative numbers', function () {
    assert.equal(numbers(-10), 'negative ten');
    assert.equal(numbers(-154), 'negative one hundred and fifty-four');
    assert.equal(numbers(-1000), 'negative one thousand');
  });

  it('should work with decimals', function () {
    assert.equal(numbers(0.5), 'zero point five');
    assert.equal(numbers(0.05), 'zero point zero five');
    assert.equal(numbers(60.5), 'sixty point five');
    assert.equal(numbers(55.2), 'fifty-five point two');
  });

  it('should handle increasingly larger numbers', function () {
    assert.equal(numbers(110), 'one hundred and ten');
    assert.equal(numbers(156), 'one hundred and fifty-six');
    assert.equal(numbers(1000), 'one thousand');
    assert.equal(numbers(1033), 'one thousand and thirty-three');
    assert.equal(numbers(1693), 'one thousand, six hundred and ninety-three');
    assert.equal(numbers(10845), 'ten thousand, eight hundred and forty-five');
    assert.equal(numbers(763405), 'seven hundred and sixty-three thousand, four hundred and five');
    assert.equal(numbers(2874595), 'two million, eight hundred and seventy-four thousand, five hundred and ninety-five');
    assert.equal(numbers(Math.pow(10, 7)), 'ten million');
    assert.equal(numbers(Math.pow(10, 9) + 162), 'one billion, one hundred and sixty-two');
    assert.equal(numbers(Math.pow(10, 10)), 'ten billion');
    assert.equal(numbers(Math.pow(10, 11)), 'one hundred billion');
    assert.equal(numbers(Math.pow(10, 11) + 6), 'one hundred billion and six');
    assert.equal(numbers(Math.pow(10, 12) + 3), 'one trillion and three');
    assert.equal(numbers(Math.pow(10, 13)), 'ten trillion');
    assert.equal(numbers(Math.pow(10, 9) + Math.pow(10, 8)), 'one billion, one hundred million');
    assert.equal(numbers(Math.pow(10, 100)), 'one googol');
  });

  it('should transform words to numbers', function () {
    assert.equal(numbers('zero'), 0);
    assert.equal(numbers('one'), 1);
    assert.equal(numbers('two'), 2);
    assert.equal(numbers('three'), 3);
    assert.equal(numbers('four'), 4);
    assert.equal(numbers('five'), 5);
    assert.equal(numbers('six'), 6);
    assert.equal(numbers('seven'), 7);
    assert.equal(numbers('eight'), 8);
    assert.equal(numbers('nine'), 9);
    assert.equal(numbers('ten'), 10);
    assert.equal(numbers('eleven'), 11);
    assert.equal(numbers('twelve'), 12);
    assert.equal(numbers('thirteen'), 13);
    assert.equal(numbers('fourteen'), 14);
    assert.equal(numbers('fifteen'), 15);
    assert.equal(numbers('sixteen'), 16);
    assert.equal(numbers('seventeen'), 17);
    assert.equal(numbers('eighteen'), 18);
    assert.equal(numbers('nineteen'), 19);
    assert.equal(numbers('twenty'), 20);
  });

  it('should transform multiple words into numbers', function () {
    assert.equal(numbers(numbers(29)), 29);
    assert.equal(numbers(numbers(36)), 36);
    assert.equal(numbers(numbers(45)), 45);
    assert.equal(numbers(numbers(51)), 51);
    assert.equal(numbers(numbers(63)), 63);
    assert.equal(numbers(numbers(78)), 78);
    assert.equal(numbers(numbers(84)), 84);
    assert.equal(numbers(numbers(92)), 92);
  });

  it('should transform more complicated number combinations', function () {
    assert.equal(numbers(numbers(122)), 122);
    assert.equal(numbers(numbers(1537)), 1537);
    assert.equal(numbers(numbers(10235)), 10235);
    assert.equal(numbers(numbers(1303457)), 1303457);
    assert.equal(numbers(numbers(832698483)), 832698483);
    assert.equal(numbers(numbers(9832798473285)), 9832798473285);
  });

  it('should handle work normally with negative numbers', function () {
    assert.equal(numbers(numbers(-833)), -833);
    assert.equal(numbers(numbers(-87365)), -87365);
    assert.equal(numbers(numbers(-9821748972)), -9821748972);
  });

  it('should work with decimals', function () {
    assert.equal(numbers(numbers(0.5)), 0.5);
    assert.equal(numbers(numbers(0.05)), 0.05);
    assert.equal(numbers(numbers(60.5)), 60.5);
    assert.equal(numbers(numbers(55.2)), 55.2);
  });

  it('should work with more human-like input', function () {
    assert.equal(numbers('zero five'), 5);
    assert.equal(numbers('five zero'), 50);
    assert.equal(numbers('zero point five'), 0.5);
    assert.equal(numbers('zero point zero five'), 0.05);
    assert.equal(numbers('two six point zero nine'), 26.09);
    assert.equal(numbers('zero zero nine five decimal two'), 95.2);
    assert.equal(numbers('zero eight zero three'), 803);
    assert.equal(numbers('eight zero three zero five'), 80305);
    assert.equal(numbers('twenty thirteen'), 2013);
    assert.equal(numbers('two decimal fifty six'), 2.56);
    assert.equal(numbers('nineteen thirty-five'), 1935);
    assert.equal(numbers('one two five six'), 1256);
    assert.equal(numbers('thirty hundred'), 3000);
  });
});
