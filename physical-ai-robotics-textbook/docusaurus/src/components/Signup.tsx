import React from 'react';
import Layout from '../theme/Layout';
import styles from '../css/auth.module.css';

export default function Signup() {
  return (
    <Layout
      title="Sign Up"
      description="Create your Humanoid Robotics account">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.authTitle}>Create Account</h1>
          <p className={styles.authSubtitle}>Join the Humanoid Robotics community</p>
          
          <form className={styles.authForm}>
            <div className={styles.formGroup}>
              <label htmlFor="name">Full Name</label>
              <input 
                type="text" 
                id="name" 
                placeholder="Enter your full name"
                required 
              />
            </div>
            
            <div className={styles.formGroup}>
              <label htmlFor="email">Email</label>
              <input 
                type="email" 
                id="email" 
                placeholder="Enter your email"
                required 
              />
            </div>
            
            <div className={styles.formGroup}>
              <label htmlFor="password">Password</label>
              <input 
                type="password" 
                id="password" 
                placeholder="Create a password"
                required 
              />
            </div>
            
            <div className={styles.formGroup}>
              <label htmlFor="confirmPassword">Confirm Password</label>
              <input 
                type="password" 
                id="confirmPassword" 
                placeholder="Confirm your password"
                required 
              />
            </div>
            
            <div className={styles.formOptions}>
              <label className={styles.checkbox}>
                <input type="checkbox" required />
                <span>I agree to the Terms & Conditions</span>
              </label>
            </div>
            
            <button type="submit" className={styles.authButton}>
              Create Account
            </button>
          </form>
          
          <div className={styles.authFooter}>
            <p>Already have an account? <a href="/login">Login</a></p>
          </div>
        </div>
      </div>
    </Layout>
  );
}