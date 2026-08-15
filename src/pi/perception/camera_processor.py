"""
============================================================
camera_processor.py — ক্যামেরা দিয়ে লাল/সবুজ pillar শনাক্তকরণ
============================================================
calibration.json এ সেভ করা HSV থ্রেশহোল্ড ব্যবহার করে ছবিতে লাল/সবুজ
ব্লক (pillar candidate) খুঁজে বের করে -- প্রতিটার bounding box,
কেন্দ্রবিন্দু (centroid), আর ক্ষেত্রফল (area) সহ।

KMIDS-এর filterColors()/extractContoursInfo() (C++) থেকে অনুপ্রাণিত,
একই মূল ধারণা Python এ:
  1. BGR -> HSV রূপান্তর
  2. HSV threshold দিয়ে বাইনারি mask বানানো (সাদা=রঙ পাওয়া গেছে)
  3. morphology দিয়ে ছোট ফাঁকফোকর বন্ধ ও দানাদার noise পরিষ্কার করা
  4. contour (সংযুক্ত সাদা অঞ্চল) খুঁজে বের করা
  5. খুব ছোট contour (ক্যামেরার সেন্সর নয়েজ, প্রতিফলন ইত্যাদি) বাদ দেওয়া
============================================================
"""

import cv2
import numpy as np


class DetectedBlock:
    """একটা শনাক্ত হওয়া রঙিন ব্লক (pillar candidate)।"""

    def __init__(self, color, x, y, w, h, area):
        self.color = color  # "red" অথবা "green"
        self.x = x           # bounding box এর উপরের-বাম কোণের x
        self.y = y
        self.w = w
        self.h = h
        self.area = area
        self.center_x = x + w // 2
        self.center_y = y + h // 2

    def __repr__(self):
        return (
            f"DetectedBlock({self.color}, center=({self.center_x},{self.center_y}), "
            f"size=({self.w}x{self.h}), area={self.area:.0f})"
        )


def _build_mask(hsv_frame, lower, upper):
    lower_np = np.array(lower, dtype=np.uint8)
    upper_np = np.array(upper, dtype=np.uint8)
    mask = cv2.inRange(hsv_frame, lower_np, upper_np)

    # morphology: OPEN ছোট দানাদার noise মুছে দেয়, CLOSE ছোট ফাঁকফোকর
    # বন্ধ করে দেয় (যাতে একটা pillar একাধিক ছোট টুকরো contour হিসেবে
    # ভেঙে না যায়)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def _merge_nearby_blocks(blocks, merge_distance=40):
    """একই রঙের কাছাকাছি ব্লকগুলোকে একটাতে জোড়া দেয়।

    কেন দরকার: কার্ড/pillar-এর উপর আলোর প্রতিফলন (glare) পড়লে সেই
    অংশটুকু threshold-এর বাইরে চলে যায়, ফলে একটাই বস্তু কয়েকটা ছোট
    বিচ্ছিন্ন contour-এ ভেঙে যেতে পারে (KMIDS এর ক্লাস্টারিং লজিকের
    মতোই ধারণা, দূরত্ব দিয়ে কাছাকাছি পয়েন্ট/ব্লক একসাথে করা)।
    """
    if len(blocks) <= 1:
        return blocks

    merged = True
    while merged:
        merged = False
        for i in range(len(blocks)):
            for j in range(i + 1, len(blocks)):
                a, b = blocks[i], blocks[j]
                dist = ((a.center_x - b.center_x) ** 2 + (a.center_y - b.center_y) ** 2) ** 0.5
                if dist < merge_distance:
                    x1 = min(a.x, b.x)
                    y1 = min(a.y, b.y)
                    x2 = max(a.x + a.w, b.x + b.w)
                    y2 = max(a.y + a.h, b.y + b.h)
                    combined = DetectedBlock(a.color, x1, y1, x2 - x1, y2 - y1, a.area + b.area)
                    blocks = [blk for k, blk in enumerate(blocks) if k not in (i, j)]
                    blocks.append(combined)
                    merged = True
                    break
            if merged:
                break
    return blocks


def detect_blocks(bgr_frame, calibration, min_area=200, top_crop_fraction=0.0, merge_distance=40):
    """bgr_frame (OpenCV BGR ছবি) থেকে লাল ও সবুজ ব্লক খুঁজে বের করে।

    calibration : config.load_calibration() থেকে পাওয়া dict
    min_area    : এর চেয়ে ছোট contour (sensor noise/প্রতিফলন) বাদ যাবে
    top_crop_fraction : ছবির উপরের এই অংশ (0.0-1.0) কালো করে দেওয়া হয়,
                  দূরের/ছাদের ভুল detection কমাতে (KMIDS এর কোডেও উপরের
                  ৫০% বাদ দেওয়া হয়েছিল, এখানে ডিফল্ট 0.0 -- আসল ট্র্যাকে
                  টেস্ট করে দরকার হলে বাড়ানো যাবে)
    merge_distance : এর চেয়ে কাছাকাছি (পিক্সেলে) একই রঙের দুটো ব্লক
                  থাকলে একটাতে জোড়া দেওয়া হবে (glare-এর কারণে ভাঙা
                  pillar একসাথে করার জন্য)

    রিটার্ন: {"red": [DetectedBlock, ...], "green": [DetectedBlock, ...]}
    (প্রতিটা লিস্ট area অনুযায়ী বড় থেকে ছোট সাজানো, সবচেয়ে বড়/কাছের
    pillar candidate সাধারণত সবচেয়ে গুরুত্বপূর্ণ)
    """
    frame = bgr_frame
    if top_crop_fraction > 0:
        frame = bgr_frame.copy()
        crop_rows = int(frame.shape[0] * top_crop_fraction)
        frame[:crop_rows, :] = 0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    results = {"red": [], "green": []}

    for color_name in ("red", "green"):
        cal = calibration[f"camera_{color_name}_hsv"]
        mask = _build_mask(hsv, cal["lower"], cal["upper"])

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        blocks = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            blocks.append(DetectedBlock(color_name, x, y, w, h, area))

        blocks = _merge_nearby_blocks(blocks, merge_distance=merge_distance)
        blocks.sort(key=lambda b: b.area, reverse=True)
        results[color_name] = blocks

    return results
